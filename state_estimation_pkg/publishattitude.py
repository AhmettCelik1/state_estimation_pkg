import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from drone_racing_msgs.msg import DroneAttitude  

class DroneAttitudeToPose(Node):
    def __init__(self):
        super().__init__('publishpose')
        
        self.subscription = self.create_subscription(
            DroneAttitude,
            '/sensors/attitude', 
            self.attitude_callback,
            5)
        
        self.publisher = self.create_publisher(PoseStamped, '/imu/ground_truth', 10)
        

    def attitude_callback(self, msg: DroneAttitude):
        pose_msg = PoseStamped()
        pose_msg.header.stamp= self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "imu_link"
        
        pose_msg.pose.position.x=6.0
        pose_msg.pose.position.y=0.0
        pose_msg.pose.position.z=0.0
        
        pose_msg.pose.orientation = msg.attitude
    
        self.publisher.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = DroneAttitudeToPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
