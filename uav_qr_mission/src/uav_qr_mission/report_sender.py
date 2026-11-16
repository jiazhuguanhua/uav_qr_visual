"""Retryable UGV display delivery; a stalled service is killed outside ROS callbacks."""
import copy
from io import BytesIO
import math
import multiprocessing
import signal
import threading
import time

import rospy

from .msg import MissionResult, QrReport, QrReportStatus
from .srv import ReportQr, ReportQrRequest


def service_worker(connection, service_name):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        connection.send('READY')
        while True:
            payload = connection.recv()
            try:
                # Resolve again after reconnect/UGV restart; don't retain an old URI.
                proxy = rospy.ServiceProxy(service_name, ReportQr)
                response = proxy(ReportQrRequest().deserialize(payload))
                connection.send((response.accepted, response.status))
            except Exception as error:
                connection.send((False, str(error)))
    except (EOFError, OSError):
        pass
    finally:
        connection.close()


class ReportSender:
    def __init__(self):
        self.service_name = rospy.get_param('~service', '/ugv/qr_display/update')
        self.timeout = float(rospy.get_param('~timeout_s', 2.0))
        self.retry = float(rospy.get_param('~retry_s', 1.0))
        self.startup_timeout = float(rospy.get_param('~startup_timeout_s', 10.0))
        self.refresh = float(rospy.get_param('~refresh_s', 10.0))
        if not all(math.isfinite(v) and 0 < v <= 30 for v in
                   (self.timeout, self.retry, self.startup_timeout, self.refresh)):
            raise ValueError('Report timeouts must be finite and in (0, 30] seconds')
        self.lock = threading.Lock()
        self.latest = None
        self.acknowledged = {}
        self.last_delivery = 0.0
        self.publisher = rospy.Publisher('~status', QrReportStatus, queue_size=1, latch=True)
        self.subscriber = rospy.Subscriber('/uav_qr_mission/result', MissionResult,
                                           self.receive, queue_size=1)
        self.context = multiprocessing.get_context('spawn')
        self.process = self.connection = None
        self.worker_ready = False
        self.start_worker()  # warm imports before the first stop produces a report

    def receive(self, message):
        with self.lock:
            if self.latest is None or message.mission_id != self.latest.mission_id:
                self.acknowledged.clear()
                self.last_delivery = 0.0
            self.latest = message

    def pending(self):
        with self.lock:
            if self.latest is None or not self.latest.mission_id:
                return None
            valid = [t for t in self.latest.targets if t.position_valid]
            changed = any(t.revision > self.acknowledged.get(t.content, 0) for t in valid)
            # A restarted screen may have lost RAM even though it previously ACKed.
            # Send full snapshots on change and periodically, without remeasuring.
            due = time.monotonic()-self.last_delivery >= self.refresh
            targets = copy.deepcopy(valid) if changed or due else []
            return QrReport(mission_id=self.latest.mission_id, targets=targets)

    def publish_status(self, status):
        with self.lock:
            if self.latest is None:
                return
            pending = [t.content for t in self.latest.targets if t.position_valid and
                       t.revision > self.acknowledged.get(t.content, 0)]
            message = QrReportStatus(mission_id=self.latest.mission_id, pending_ids=pending,
                                    acknowledged_ids=sorted(self.acknowledged), status=status)
        self.publisher.publish(message)

    def stop_worker(self):
        if self.connection is not None:
            self.connection.close()
        if self.process is not None:
            if self.process.is_alive():
                self.process.terminate()
            self.process.join(timeout=0.2)
            if self.process.is_alive():
                self.process.kill()
                self.process.join()
        self.process = self.connection = None
        self.worker_ready = False

    def start_worker(self):
        if self.process is not None and self.process.is_alive():
            return
        self.stop_worker()
        parent, child = self.context.Pipe()
        process = self.context.Process(target=service_worker,
                                       args=(child, self.service_name), daemon=True)
        try:
            process.start()
        except BaseException:
            parent.close()
            raise
        finally:
            child.close()
        self.process, self.connection = process, parent

    def deliver(self, report):
        try:
            self.start_worker()
            # Import/worker startup is separate from the actual service deadline.
            # Keeping this process warm makes one-target updates prompt on Jetson.
            startup_deadline = time.monotonic()+self.startup_timeout
            while not self.worker_ready and not rospy.is_shutdown():
                if time.monotonic() >= startup_deadline:
                    self.stop_worker()
                    return False, 'WORKER_START_TIMEOUT'
                if self.connection.poll(0.05):
                    self.worker_ready = self.connection.recv() == 'READY'
            if rospy.is_shutdown():
                return False, 'SHUTDOWN'
            buffer = BytesIO()
            ReportQrRequest(report=report).serialize(buffer)
            self.connection.send(buffer.getvalue())
            deadline = time.monotonic() + self.timeout
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                if self.connection.poll(min(0.05, max(0, deadline-time.monotonic()))):
                    return self.connection.recv()
            self.stop_worker()
            return False, 'SERVICE_TIMEOUT'
        except EOFError:
            self.stop_worker()
            return False, 'WORKER_EXITED_WITHOUT_RESPONSE'
        except OSError as error:
            self.stop_worker()
            return False, str(error)

    def run(self):
        while not rospy.is_shutdown():
            report = self.pending()
            if report is not None and report.targets:
                accepted, status = self.deliver(report)
                with self.lock:
                    # Never acknowledge an old task against a newer task's registry.
                    if accepted and self.latest.mission_id == report.mission_id:
                        self.last_delivery = time.monotonic()
                        for target in report.targets:
                            self.acknowledged[target.content] = max(
                                target.revision, self.acknowledged.get(target.content, 0))
                self.publish_status('ACKNOWLEDGED' if accepted else status)
                if not accepted:
                    rospy.logwarn_throttle(5, 'UGV QR display pending: %s', status)
            else:
                self.publish_status('IDLE')
            # Monotonic, shutdown-aware delay; a paused /clock must not stop retries.
            deadline = time.monotonic() + self.retry
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                time.sleep(0.05)


def main():
    rospy.init_node('qr_report_sender')
    sender = ReportSender()
    try:
        sender.run()
    finally:
        sender.stop_worker()
