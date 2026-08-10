#!/usr/bin/env python3
"""Publish keyboard velocity commands using the original Dashgo bindings."""

import select
import sys
import termios
import tty

import rospy
from geometry_msgs.msg import Twist


HELP = """
Reading from the keyboard and publishing Twist commands.
--------------------------------------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

For holonomic mode (strafing), hold down the shift key:
   U    I    O
   J    K    L
   M    <    >

t/b: up/down (+/- z)
anything else: stop
q/z: increase/decrease max speeds by 10%
w/x: increase/decrease only linear speed by 10%
e/c: increase/decrease only angular speed by 10%
CTRL-C to quit
"""

MOVE_BINDINGS = {
    "i": (1, 0, 0, 0),
    "o": (1, 0, 0, -1),
    "j": (0, 0, 0, 1),
    "l": (0, 0, 0, -1),
    "u": (1, 0, 0, 1),
    ",": (-1, 0, 0, 0),
    ".": (-1, 0, 0, 1),
    "m": (-1, 0, 0, -1),
    "O": (1, -1, 0, 0),
    "I": (1, 0, 0, 0),
    "J": (0, 1, 0, 0),
    "L": (0, -1, 0, 0),
    "U": (1, 1, 0, 0),
    "<": (-1, 0, 0, 0),
    ">": (-1, -1, 0, 0),
    "M": (-1, 1, 0, 0),
    "t": (0, 0, 1, 0),
    "b": (0, 0, -1, 0),
}

SPEED_BINDINGS = {
    "q": (1.1, 1.1),
    "z": (0.9, 0.9),
    "w": (1.1, 1.0),
    "x": (0.9, 1.0),
    "e": (1.0, 1.1),
    "c": (1.0, 0.9),
}


def get_key(settings):
    """Read one key and restore the terminal immediately."""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def velocity_text(speed, turn):
    """Format the current velocity limits."""
    return "currently:\tspeed %s\tturn %s" % (speed, turn)


def publish_twist(publisher, x, y, z, th, speed, turn):
    """Publish one Twist using the selected binding and limits."""
    twist = Twist()
    twist.linear.x = x * speed
    twist.linear.y = y * speed
    twist.linear.z = z * speed
    twist.angular.z = th * turn
    publisher.publish(twist)


def main():
    """Run the keyboard loop until Ctrl-C or ROS shutdown."""
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("teleop_twist_keyboard")
    publisher = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    speed = 0.30
    turn = 0.6
    x = y = z = th = 0
    status = 0

    try:
        print(HELP)
        print(velocity_text(speed, turn))
        while not rospy.is_shutdown():
            key = get_key(settings)
            if key in MOVE_BINDINGS:
                x, y, z, th = MOVE_BINDINGS[key]
            elif key in SPEED_BINDINGS:
                speed *= SPEED_BINDINGS[key][0]
                turn *= SPEED_BINDINGS[key][1]
                print(velocity_text(speed, turn))
                if status == 14:
                    print(HELP)
                status = (status + 1) % 15
            else:
                x = y = z = th = 0
                if key == "\x03":
                    break
            publish_twist(publisher, x, y, z, th, speed, turn)
    except Exception as exc:  # Terminal errors must still trigger a stop command.
        rospy.logerr("Keyboard teleop failed: %s", exc)
    finally:
        publisher.publish(Twist())
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
