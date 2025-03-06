import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Vector3Stamped
from tf_transformations import euler_from_quaternion
import math 

class PoseToEulerNode(Node):
    def __init__(self):
        super().__init__('pose_to_euler_node')

        self.declare_parameter('topic_name', '/pose_topic')

        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            PoseStamped,
            topic_name,
            self.pose_callback,
            5)
        
        pub_toic_name=topic_name +"_euler"
        self.publisher_rpy = self.create_publisher(Vector3Stamped, pub_toic_name, 5)
    

    def pose_callback(self, msg):
        orientation_q = msg.pose.orientation
        quaternion = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

        (roll, pitch, yaw) = euler_from_quaternion(quaternion)

        roll_deg = roll * 180.0 / math.pi
        pitch_deg = pitch * 180.0 / math.pi
        yaw_deg = yaw * 180.0 / math.pi
        
        rpy=Vector3Stamped()
        rpy.header.frame_id="imu_link"
        rpy.header.stamp =msg.header.stamp
        rpy.vector.x=roll_deg
        rpy.vector.y=pitch_deg
        rpy.vector.z=yaw_deg
        
        self.publisher_rpy.publish(rpy)
        
def main(args=None):
    rclpy.init(args=args)

    pose_to_euler_node = PoseToEulerNode()

    rclpy.spin(pose_to_euler_node)

    pose_to_euler_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()