"""Map GPS epoch to ROS time through FCU boot time; never set either clock."""
from collections import deque
import math
import time


class ClockBridge:
    def __init__(self):
        self.anchors = deque(maxlen=5)
        self.sync = None

    def update_sync(self, offset_ns, rtt_ms):
        if not math.isfinite(rtt_ms) or not 0 <= rtt_ms <= 10:
            return
        now = time.monotonic()
        if self.sync and (now-self.sync[0] > 3 or abs(offset_ns-self.sync[1]) > 50000000):
            self.anchors.clear()
        self.sync = (now, int(offset_ns))

    def update_system(self, epoch_us, boot_ms):
        if epoch_us < 1000000000000000 or boot_ms <= 0:
            return
        now = time.monotonic()
        anchor = int(epoch_us)*1000-int(boot_ms)*1000000
        # Reboot, UTC step or inconsistent SYSTEM_TIME: require convergence again.
        if self.anchors and (now-self.anchors[-1][0] > 5 or
                             abs(anchor-self.anchors[-1][1]) > 50000000):
            self.anchors.clear()
        self.anchors.append((now, anchor))

    def offset(self):
        now = time.monotonic()
        if self.sync is None or now-self.sync[0] > 3:
            return None, 'WAIT_CLOCK_SYNC'
        if len(self.anchors) < 3 or now-self.anchors[-1][0] > 5:
            return None, 'WAIT_CLOCK_ANCHOR'
        # ROS = boot + TIMESYNC; GPS epoch = boot + SYSTEM_TIME anchor.
        anchors = sorted(a[1] for a in self.anchors)
        return self.sync[1]-anchors[len(anchors)//2], 'VALID'
