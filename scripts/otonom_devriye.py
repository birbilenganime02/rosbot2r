#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class OtonomSurus(Node):
    def __init__(self):
        super().__init__('otonom_surus')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.hedef_x = 9.0

    def cb(self, msg):
        x = msg.pose.pose.position.x
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if abs(self.hedef_x - x) < 0.6:
            self.hedef_x = -9.0 if self.hedef_x > 0 else 9.0
        hedef_yaw = 0.0 if self.hedef_x > x else math.pi
        err = (hedef_yaw - yaw + math.pi) % (2.0 * math.pi) - math.pi
        t = Twist()
        if abs(err) > 0.25:
            t.angular.z = 0.8 if err > 0 else -0.8
        else:
            t.linear.x = 0.45
            t.angular.z = 1.2 * err
        self.pub.publish(t)

def main():
    rclpy.init()
    node = OtonomSurus()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
