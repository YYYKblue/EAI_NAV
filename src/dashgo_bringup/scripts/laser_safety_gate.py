#!/usr/bin/env python3
"""Publish the number of scan points inside a configured base-frame box."""

import math

import rospy
import tf
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int16
from tf.transformations import quaternion_matrix


class LaserSafetyGate:
    """Transform scan points into the robot frame and count box intrusions."""

    def __init__(self):
        self.box_frame = rospy.get_param("~box_frame", "base_footprint")
        self.min_x = rospy.get_param("~min_x")
        self.max_x = rospy.get_param("~max_x")
        self.min_y = rospy.get_param("~min_y")
        self.max_y = rospy.get_param("~max_y")
        self.min_z = rospy.get_param("~min_z")
        self.max_z = rospy.get_param("~max_z")

        self.listener = tf.TransformListener()
        self.publisher = rospy.Publisher("is_passed", Int16, queue_size=1)
        self.subscriber = rospy.Subscriber(
            "scan", LaserScan, self.scan_callback, queue_size=1
        )

    def scan_callback(self, scan):
        """Count scan returns within the configured box and publish the count."""
        stamp = scan.header.stamp if scan.header.stamp != rospy.Time() else rospy.Time(0)
        try:
            translation, rotation = self.listener.lookupTransform(
                self.box_frame, scan.header.frame_id, stamp
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            rospy.logwarn_throttle(5.0, "Laser safety TF unavailable: %s", exc)
            return

        matrix = quaternion_matrix(rotation)
        intrusions = 0
        angle = scan.angle_min
        for distance in scan.ranges:
            if math.isfinite(distance) and scan.range_min <= distance <= scan.range_max:
                laser_x = distance * math.cos(angle)
                laser_y = distance * math.sin(angle)
                base_x = (
                    matrix[0][0] * laser_x
                    + matrix[0][1] * laser_y
                    + translation[0]
                )
                base_y = (
                    matrix[1][0] * laser_x
                    + matrix[1][1] * laser_y
                    + translation[1]
                )
                base_z = (
                    matrix[2][0] * laser_x
                    + matrix[2][1] * laser_y
                    + translation[2]
                )
                if (
                    self.min_x <= base_x <= self.max_x
                    and self.min_y <= base_y <= self.max_y
                    and self.min_z <= base_z <= self.max_z
                ):
                    intrusions += 1
            angle += scan.angle_increment

        self.publisher.publish(Int16(data=min(intrusions, 32767)))


def main():
    """Start one configurable scan safety gate."""
    rospy.init_node("laser_safety_gate")
    LaserSafetyGate()
    rospy.spin()


if __name__ == "__main__":
    main()
