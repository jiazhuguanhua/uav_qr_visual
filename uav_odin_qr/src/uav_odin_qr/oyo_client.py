"""Bound a blocking rospy service call without registering a second ROS node."""

import multiprocessing
import signal
import time
from io import BytesIO


def _query_worker(connection, service_name, caller):
    # roslaunch sends SIGINT to the process group; the node owns worker cleanup.
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    import rospy
    from uav_oyo.srv import QueryPixel

    proxy = rospy.ServiceProxy(service_name, QueryPixel, headers={'callerid': caller})
    with connection:
        while True:
            try:
                u, v, secs, nsecs = connection.recv()
            except EOFError:
                return
            try:
                response = proxy(u, v, rospy.Time(secs, nsecs))
                buffer = BytesIO()
                response.serialize(buffer)
                connection.send((buffer.getvalue(), ''))
            except (rospy.ServiceException, OSError) as error:
                connection.send((None, 'SERVICE_UNAVAILABLE: ' + str(error)))


class OyoClient:
    def __init__(self, service_name, caller):
        self.service_name = service_name
        self.caller = caller
        self.context = multiprocessing.get_context('spawn')
        self.process = self.connection = None

    def start(self):
        if self.process is not None and self.process.is_alive():
            return
        self.close()
        parent, child = self.context.Pipe()
        process = self.context.Process(
            target=_query_worker, args=(child, self.service_name, self.caller), daemon=True
        )
        try:
            process.start()
        except BaseException:
            parent.close()
            raise
        finally:
            child.close()
        self.connection, self.process = parent, process

    def call(self, u, v, stamp, deadline):
        if time.monotonic() >= deadline:
            return None, 'QUERY_TIMEOUT'
        try:
            self.start()
            self.connection.send((u, v, stamp.secs, stamp.nsecs))
            remaining = max(0.0, deadline - time.monotonic())
            if self.connection.poll(remaining):
                result = self.connection.recv()
                if time.monotonic() <= deadline:
                    return result
            self.close()
            return None, 'QUERY_TIMEOUT'
        except (EOFError, OSError) as error:
            self.close()
            return None, 'SERVICE_UNAVAILABLE: ' + str(error)

    def close(self):
        if self.connection is not None:
            self.connection.close()
        if self.process is not None:
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(0.05)
                if self.process.is_alive():
                    self.process.kill()
            self.process.join()
            self.process.close()
        self.connection = self.process = None
