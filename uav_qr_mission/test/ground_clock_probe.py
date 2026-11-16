#!/usr/bin/env python3
"""Read-only clock validation; never publishes or calls aircraft services."""
import json
import struct
import threading
import time

import rospy
from mavros_msgs.msg import Mavlink, TimesyncStatus, State
from sensor_msgs.msg import NavSatFix, Imu, Image
from uav_qr_mission.clock_bridge import ClockBridge

rospy.init_node('qr_clock_bridge_readonly', anonymous=True)
bridge, lock = ClockBridge(), threading.Lock()
data = {'fix_count': 0, 'imu_count': 0, 'image_count': 0, 'armed_seen': False}

def sync(m):
    with lock:
        bridge.update_sync(m.estimated_offset_ns, m.round_trip_time_ms)

def system(m):
    if m.msgid == 2 and m.sysid == 1 and m.compid == 1 and m.framing_status == m.FRAMING_OK:
        payload = b''.join(struct.pack('<Q', v) for v in m.payload64).ljust(12, b'\0')
        with lock:
            bridge.update_system(*struct.unpack_from('<QI', payload))

def sensor(kind, m):
    with lock:
        data[kind+'_count'] += 1
        if kind == 'fix':
            offset, status = bridge.offset()
            data['clock_status'] = status
            if offset is not None:
                data['gps_to_ros_offset_s'] = offset/1e9
                data['mapped_fix_minus_receive_s'] = (m.header.stamp.to_nsec()+offset-rospy.Time.now().to_nsec())/1e9

def state(m):
    with lock:
        data['armed_seen'] |= m.armed
        data['state'] = {'connected': m.connected, 'armed': m.armed, 'mode': m.mode}

subs = [rospy.Subscriber('/mavros/timesync_status', TimesyncStatus, sync),
        rospy.Subscriber('/mavlink/from', Mavlink, system, queue_size=100),
        rospy.Subscriber('/mavros/state', State, state)]
for kind, topic, cls in [('fix','/mavros/global_position/raw/fix',NavSatFix),
                         ('imu','/mavros/imu/data',Imu),
                         ('image','/odin1/image/undistorted',Image)]:
    subs.append(rospy.Subscriber(topic,cls,lambda m,k=kind:sensor(k,m),queue_size=1))
for _ in range(4):
    time.sleep(5)
    with lock:
        print(json.dumps(data,sort_keys=True),flush=True)
