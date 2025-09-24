#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleLocalPosition

class QoSRelay(Node):
    def __init__(self):
        super().__init__('qos_relay_node')
        
        # QoS for subscribing to PX4 (best_effort)
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS for publishing to Unity (reliable)
        unity_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to PX4 topic with best_effort
        self.subscription = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.relay_callback,
            px4_qos
        )
        
        # Publish for Unity with reliable
        self.publisher = self.create_publisher(
            VehicleLocalPosition,
            '/unity/vehicle_local_position',
            unity_qos
        )
        
        self.get_logger().info('QoS Relay started: PX4 -> Unity')
        self.msg_count = 0
        
    def relay_callback(self, msg):
        self.publisher.publish(msg)
        self.msg_count += 1
        if self.msg_count % 50 == 0:
            self.get_logger().info(f'Relayed {self.msg_count} messages')

def main(args=None):
    rclpy.init(args=args)
    node = QoSRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
