#!/usr/bin/env python3
"""Display delivery/reconnect contract; isolated master, no flight interfaces."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from urllib.parse import urlparse

import rospy
from uav_qr_mission.msg import MissionResult, QrTargetResult, QrReportStatus
from uav_qr_mission.srv import ReportQr, ReportQrResponse


def guard():
    uri = urlparse(os.environ.get('ROS_MASTER_URI',''))
    if uri.hostname not in ('localhost','127.0.0.1') or uri.port == 11311:
        raise RuntimeError('Use a dedicated loopback test master')


def receiver(path):
    rospy.init_node('report_contract_receiver',anonymous=True)
    def receive(request):
        entry = {'mission_id':request.report.mission_id,
                 'contents':[t.content for t in request.report.targets],
                 'stamps':[t.position.header.stamp.to_nsec() for t in request.report.targets]}
        with path.open('a') as output:
            output.write(json.dumps(entry)+'\n')
        return ReportQrResponse(True,'SAVED')
    service = rospy.Service('/ugv/qr_display/update',ReportQr,receive)
    rospy.spin()


def stop(process):
    if process.poll() is None:
        os.killpg(process.pid,signal.SIGINT)
        try: process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid,signal.SIGKILL)
            process.wait()


def contract(path):
    rospy.init_node('report_contract')
    latest = [None]
    def status(message): latest[0] = message
    subscriber = rospy.Subscriber('/qr_report_sender/status',QrReportStatus,status)
    publisher = rospy.Publisher('/uav_qr_mission/result',MissionResult,queue_size=1,latch=True)
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text('')
    output = path.with_suffix('.log').open('w')
    backend = None
    sender = subprocess.Popen(['rosrun','uav_qr_mission','qr_report_sender','_refresh_s:=2.0'],
                              stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
    def start_receiver():
        return subprocess.Popen([sys.executable,__file__,'--receiver','--output',str(path)],
                                stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
    def wait(predicate,timeout=20):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if predicate(): return
            if sender.poll() is not None: raise AssertionError('Sender exited')
            time.sleep(.05)
        raise AssertionError('Report delivery timeout; latest={}'.format(latest[0]))
    try:
        backend = start_receiver()
        wait(lambda: publisher.get_num_connections() > 0)
        message = MissionResult(mission_id='report_reconnect_contract')
        for content in ('A','B','C'):
            target = QrTargetResult(content=content,found=True,position_valid=True,revision=1,status='VALID')
            target.position.header.stamp = rospy.Time.now()
            target.position.header.frame_id = 'wgs84'
            target.position.latitude,target.position.longitude,target.position.altitude = 31.2,121.4,100
            message.targets.append(target)
        first = MissionResult(mission_id=message.mission_id,targets=message.targets[:1])
        publisher.publish(first)
        wait(lambda: latest[0] is not None and latest[0].acknowledged_ids == ['A'])
        stop(backend)
        backend = start_receiver()
        # No new detections: periodic full snapshots must rebuild a restarted screen.
        wait(lambda: sum(json.loads(line)['contents'] == ['A']
                         for line in path.read_text().splitlines()) >= 2)
        stop(backend)
        publisher.publish(message)
        wait(lambda: latest[0] is not None and set(latest[0].pending_ids) == {'B','C'})
        # A new process has a new ROS service URI. The warm sender must resolve it again.
        backend = start_receiver()
        wait(lambda: latest[0] is not None and set(latest[0].acknowledged_ids) == {'A','B','C'})
        entries = [json.loads(line) for line in path.read_text().splitlines()]
        assert any(e['contents'] == ['A'] for e in entries), entries
        assert any(set(e['contents']) == {'A','B','C'} for e in entries), entries
        assert all(stamp > 0 for e in entries for stamp in e['stamps'])
        print('PASS: single report, batch report, receiver restart/URI refresh:',entries)
    finally:
        stop(sender)
        if backend is not None: stop(backend)
        output.close()
        rospy.signal_shutdown('Report contract complete')


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--receiver',action='store_true')
    parser.add_argument('--output',required=True,type=Path)
    options = parser.parse_args(rospy.myargv()[1:])
    guard()
    if options.receiver: receiver(options.output)
    else: contract(options.output)
