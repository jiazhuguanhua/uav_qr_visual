#!/usr/bin/env python3
"""Label current QR observations in RViz without publishing any TF."""
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from uav_odin_qr.msg import QrTransformArray


def main():
    rospy.init_node('qr_rviz_markers')
    publisher = rospy.Publisher('~markers', MarkerArray, queue_size=1)

    def receive(message):
        clear = Marker(action=Marker.DELETEALL)
        clear.header.frame_id = 'uav_camera_optical_frame'
        markers = [clear]
        for index, target in enumerate(message.targets):
            for label in (False, True):
                marker = Marker()
                marker.header = target.transform.header
                marker.ns = 'qr_labels' if label else 'qr_centers'
                marker.id = index
                marker.type = Marker.TEXT_VIEW_FACING if label else Marker.SPHERE
                marker.action = Marker.ADD
                point = target.transform.transform.translation
                marker.pose.position.x = point.x
                marker.pose.position.y = point.y - (0.18 if label else 0)
                marker.pose.position.z = point.z
                marker.pose.orientation.w = 1.0
                marker.scale.x = marker.scale.y = 0.10
                marker.scale.z = 0.14 if label else 0.10
                marker.color.r = 1.0 if label else 0.1
                marker.color.g = 1.0
                marker.color.b = 1.0 if label else 0.3
                marker.color.a = 1.0
                marker.lifetime = rospy.Duration(0.7)
                marker.text = '{}\nX={:.2f} Y={:.2f} Z={:.2f} m'.format(
                    target.content, point.x, point.y, point.z)
                markers.append(marker)
        publisher.publish(MarkerArray(markers=markers))

    rospy.Subscriber('/uav_odin_qr/tf_targets', QrTransformArray, receive, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
