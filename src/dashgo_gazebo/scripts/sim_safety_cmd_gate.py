#!/usr/bin/env python3
"""Apply the real driver's two forward-motion safety gates in simulation."""

import threading

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16


class SimSafetyCmdGate:
    """Forward smoothed commands to Gazebo after applying the hardware rule."""

    def __init__(self):
        self._lock = threading.Lock()
        self._front_clear = True
        self._lower_front_clear = True
        self._last_command = rospy.Time.now()
        self._timed_out = False
        self._timeout = rospy.Duration(rospy.get_param("~command_timeout", 1.0))

        self._gazebo_pub = rospy.Publisher("gazebo_cmd_vel", Twist, queue_size=1)
        # The real driver publishes the requested command on this diagnostic topic.
        self._robot_cmd_pub = rospy.Publisher("robot_cmd_vel", Twist, queue_size=1)

        rospy.Subscriber("smoother_cmd_vel", Twist, self._command_callback, queue_size=1)
        rospy.Subscriber("is_passed", Int16, self._front_callback, queue_size=1)
        rospy.Subscriber("is_passed_2", Int16, self._lower_front_callback, queue_size=1)
        rospy.Timer(rospy.Duration(0.05), self._watchdog_callback)

    @staticmethod
    def _copy_twist(source):
        target = Twist()
        target.linear.x = source.linear.x
        target.linear.y = source.linear.y
        target.linear.z = source.linear.z
        target.angular.x = source.angular.x
        target.angular.y = source.angular.y
        target.angular.z = source.angular.z
        return target

    def _front_callback(self, message):
        with self._lock:
            self._front_clear = message.data <= 2

    def _lower_front_callback(self, message):
        with self._lock:
            self._lower_front_clear = message.data <= 2

    def _command_callback(self, message):
        requested = self._copy_twist(message)
        accepted = self._copy_twist(message)
        with self._lock:
            if accepted.linear.x > 0.0 and not (
                self._front_clear and self._lower_front_clear
            ):
                accepted.linear.x = 0.0
            self._last_command = rospy.Time.now()
            self._timed_out = False

        self._robot_cmd_pub.publish(requested)
        self._gazebo_pub.publish(accepted)

    def _watchdog_callback(self, _event):
        with self._lock:
            timed_out = rospy.Time.now() - self._last_command > self._timeout
            publish_stop = timed_out and not self._timed_out
            self._timed_out = timed_out
        if publish_stop:
            self._gazebo_pub.publish(Twist())


def main():
    rospy.init_node("sim_safety_cmd_gate")
    SimSafetyCmdGate()
    rospy.spin()


if __name__ == "__main__":
    main()

