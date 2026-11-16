#!/usr/bin/env python3
"""Synthetic clock skew test, isolated loopback master only, no aircraft nodes."""
import os
import struct
import time
from urllib.parse import urlparse

import rospy
from mavros_msgs.msg import Mavlink, TimesyncStatus
from sensor_msgs.msg import Imu, NavSatFix
from uav_qr_mission.msg import MissionResult

uri = urlparse(os.environ['ROS_MASTER_URI'])
if uri.hostname not in ('127.0.0.1', 'localhost') or uri.port == 11311:
    raise RuntimeError('Isolated loopback master required')
rospy.init_node('clock_contract', anonymous=True)
latest = []
sub = rospy.Subscriber('/uav_qr_mission/result', MissionResult, lambda m: latest.append(m))
pubs = [rospy.Publisher(t,c,queue_size=10) for t,c in [
    ('/mavlink/from',Mavlink),('/mavros/timesync_status',TimesyncStatus),
    ('/mavros/global_position/raw/fix',NavSatFix),('/mavros/imu/data',Imu)]]
deadline = time.monotonic()+5
while not all(p.get_num_connections() for p in pubs):
    if time.monotonic()>deadline: raise RuntimeError('Missing mission subscribers')
    time.sleep(.05)
boot_offset = rospy.Time.now().to_nsec()-40000000000
for _ in range(30):
    now = rospy.Time.now()
    sync = TimesyncStatus(estimated_offset_ns=boot_offset,round_trip_time_ms=2)
    payload = struct.pack('<QI',(now.to_nsec()+800000000)//1000,(now.to_nsec()-boot_offset)//1000000)
    wire = Mavlink(framing_status=Mavlink.FRAMING_OK,sysid=1,compid=1,msgid=2,len=12,
                   payload64=list(struct.unpack('<QQ',payload.ljust(16,b'\0'))))
    fix = NavSatFix()
    fix.header.stamp = now+rospy.Duration(.8)
    fix.latitude,fix.longitude,fix.altitude = 0,0,100
    imu = Imu()
    imu.header.stamp = now
    imu.orientation.w = 1
    for p,m in zip(pubs,[wire,sync,fix,imu]): p.publish(m)
    time.sleep(.1)
assert latest and latest[-1].geolocation_input_status == 'VALID', latest[-1] if latest else 'No result'
print('PASS: +0.8 s GPS clock skew maps to VALID through ROS subscriptions')
time.sleep(4)
assert latest[-1].geolocation_input_status == 'WAIT_CLOCK_SYNC', latest[-1]
print('PASS: expired time mapping rejects localization')
