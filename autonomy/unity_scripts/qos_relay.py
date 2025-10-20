#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import VehicleLocalPosition
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q

class Px4ToOdomRelay(Node):
    def __init__(self):
        super().__init__('px4_to_odom_relay')

        # PX4 subscribe QoS: best_effort
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position',
            self.cb, px4_qos
        )
        # realse Unity: /drone/odom
        self.pub = self.create_publisher(Odometry, '/drone/odom', 10)
        self.count = 0
        self.get_logger().info('PX4 VehicleLocalPosition -> /drone/odom (NED) relay started')

    def cb(self, msg: VehicleLocalPosition):
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'map_ned'      # NED
        odom.child_frame_id = 'base_link_ned'

        # NED）：x=N, y=E, z=D
        odom.pose.pose.position.x = float(msg.x)
        odom.pose.pose.position.y = float(msg.y)
        odom.pose.pose.position.z = float(msg.z)

        # PX4 heading  NED-yaw(radians, north is 0, clockwise is positive); set to 0 if missing
        yaw = float(msg.heading) if hasattr(msg, 'heading') and not math.isnan(msg.heading) else 0.0
        odom.pose.pose.orientation = yaw_to_quat(yaw)

        # speed(optional)
        odom.twist.twist.linear.x = float(msg.vx)
        odom.twist.twist.linear.y = float(msg.vy)
        odom.twist.twist.linear.z = float(msg.vz)

        self.pub.publish(odom)
        self.count += 1
        if self.count % 50 == 0:
            self.get_logger().info(f'Published {self.count} /drone/odom msgs')

def main(args=None):
    rclpy.init(args=args)
    node = Px4ToOdomRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
