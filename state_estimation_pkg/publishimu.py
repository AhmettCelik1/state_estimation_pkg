import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class IMUTimestampUpdater(Node):
    def __init__(self):
        super().__init__('imu_timestamp_updater')
        self.subscription = self.create_subscription(
            Imu,
            '/sensors/imu',
            self.imu_callback,
            5)
        self.publisher = self.create_publisher(Imu, '/sensors/imu_with_timestamp', 5)

    def imu_callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = IMUTimestampUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
