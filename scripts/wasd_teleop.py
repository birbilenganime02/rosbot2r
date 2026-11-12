#!/usr/bin/env python3
import sys, termios, tty, rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

MSG = """
--- ROSBOT2R WASD KONTROL ---
w : Ileri duz
s : Geri duz
a : Sola don
d : Saga don
k / Bosluk : Dur
q : Cikis
"""

def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def main():
    rclpy.init()
    node = rclpy.create_node('wasd_teleop')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    print(MSG)
    twist = Twist()
    try:
        while True:
            k = get_key().lower()
            if k == 'w':
                twist.linear.x, twist.angular.z = 0.35, 0.0
            elif k == 's':
                twist.linear.x, twist.angular.z = -0.35, 0.0
            elif k == 'a':
                twist.linear.x, twist.angular.z = 0.0, 0.8
            elif k == 'd':
                twist.linear.x, twist.angular.z = 0.0, -0.8
            elif k in ('k', ' '):
                twist.linear.x, twist.angular.z = 0.0, 0.0
            elif k == 'q' or k == '\x03':
                twist.linear.x, twist.angular.z = 0.0, 0.0
                pub.publish(twist)
                break
            pub.publish(twist)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
