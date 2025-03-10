#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from tf_transformations import euler_from_quaternion

SPIN_RATE_LIMIT = 20

class IMUPoseEstimator(Node):
    def __init__(self):
        super().__init__('imu_pose_estimator')
        
        self.q_w, self.q_x, self.q_y, self.q_z = 1.0, 0.0, 0.0, 0.0

        # Mahony filter gains
        self.Kp = 0.00  
        self.Ki = 0.00
        self.integralFBx = 0.0
        self.integralFBy = 0.0
        self.integralFBz = 0.0
        
        self.headingErrCog = 0.0
        self.headingErrMag = 0.0
        
        self.useAcc = True
        
        self.rMat = [[0.0]*3 for _ in range(3)]
        
        self.dt = 0.01
        
        self.pose_pub = self.create_publisher(PoseStamped, '/imu/mahony_ahrs/pose', 10)
        
        self.subscription = self.create_subscription(Imu,'/sensors/imu', self.imu_callback,10)
    
    def imu_callback(self, msg):
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z

        self.imuMahonyAHRSupdate(self.dt, gx, gy, gz, self.useAcc, ax, ay, az, self.headingErrMag, self.headingErrCog, self.Kp)
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "imu_link"
        
        pose_msg.pose.position.x=-2.0
        pose_msg.pose.position.y=0.0
        pose_msg.pose.position.z=0.0
        
        
        pose_msg.pose.orientation.w = self.q_w
        pose_msg.pose.orientation.x = self.q_x
        pose_msg.pose.orientation.y = self.q_y
        pose_msg.pose.orientation.z = self.q_z
        self.pose_pub.publish(pose_msg)
        
        quaternion = [self.q_x, self.q_y, self.q_z, self.q_w]
        
        # euler = euler_from_quaternion(quaternion)
        # roll, pitch, yaw = euler
        # self.get_logger().info(f"Updated Euler Angles: yaw: {yaw:.4f}, pitch: {pitch:.4f}, roll: {roll:.4f}")
    
    def imuMahonyAHRSupdate(self, dt, gx, gy, gz, useAcc, ax, ay, az, headingErrMag, headingErrCog, dcmKpGain):
        spin_rate = math.sqrt(gx**2 + gy**2 + gz**2)
        
        ex, ey, ez = 0.0, 0.0, 0.0
        ex += self.rMat[2][0] * (headingErrCog + headingErrMag)
        ey += self.rMat[2][1] * (headingErrCog + headingErrMag)
        ez += self.rMat[2][2] * (headingErrCog + headingErrMag)
        
        recipAccNorm = ax**2 + ay**2 + az**2
        if useAcc and recipAccNorm > 0.01:
            recipAccNorm = 1.0 / math.sqrt(recipAccNorm)
            ax *= recipAccNorm
            ay *= recipAccNorm
            az *= recipAccNorm
            
            ex += (ay * self.rMat[2][2] - az * self.rMat[2][1])
            ey += (az * self.rMat[2][0] - ax * self.rMat[2][2])
            ez += (ax * self.rMat[2][1] - ay * self.rMat[2][0])
        
        if self.Ki > 0.0 and spin_rate < math.radians(SPIN_RATE_LIMIT):
            self.integralFBx += self.Ki * ex * dt
            self.integralFBy += self.Ki * ey * dt
            self.integralFBz += self.Ki * ez * dt
        else:
            self.integralFBx, self.integralFBy, self.integralFBz = 0.0, 0.0, 0.0
        
        gx += dcmKpGain * ex + self.integralFBx
        gy += dcmKpGain * ey + self.integralFBy
        gz += dcmKpGain * ez + self.integralFBz
        
        gx *= 0.5 * dt
        gy *= 0.5 * dt
        gz *= 0.5 * dt
        
        buffer_w, buffer_x, buffer_y, buffer_z = self.q_w, self.q_x, self.q_y, self.q_z
        
        self.q_w += (-buffer_x * gx - buffer_y * gy - buffer_z * gz)
        self.q_x += (buffer_w * gx + buffer_y * gz - buffer_z * gy)
        self.q_y += (buffer_w * gy - buffer_x * gz + buffer_z * gx)
        self.q_z += (buffer_w * gz + buffer_x * gy - buffer_y * gx)
        
        recipNorm = 1.0 / math.sqrt(self.q_w**2 + self.q_x**2 + self.q_y**2 + self.q_z**2)
        self.q_w *= recipNorm
        self.q_x *= recipNorm
        self.q_y *= recipNorm
        self.q_z *= recipNorm
        
        self.imuComputeRotationMatrix()
    
    def imuComputeRotationMatrix(self):
        self.rMat[0][0] = 1.0 - 2.0 * (self.q_y**2 + self.q_z**2)
        self.rMat[0][1] = 2.0 * (self.q_x * self.q_y - self.q_w * self.q_z)
        self.rMat[0][2] = 2.0 * (self.q_x * self.q_z + self.q_w * self.q_y)
        
        self.rMat[1][0] = 2.0 * (self.q_x * self.q_y + self.q_w * self.q_z)
        self.rMat[1][1] = 1.0 - 2.0 * (self.q_x**2 + self.q_z**2)
        self.rMat[1][2] = 2.0 * (self.q_y * self.q_z - self.q_w * self.q_x)
        
        self.rMat[2][0] = 2.0 * (self.q_x * self.q_z - self.q_w * self.q_y)
        self.rMat[2][1] = 2.0 * (self.q_y * self.q_z + self.q_w * self.q_x)
        self.rMat[2][2] = 1.0 - 2.0 * (self.q_x**2 + self.q_y**2)
    
    def run(self):
        rclpy.spin(self)

def main(args=None):
    rclpy.init(args=args)
    imu_pose_estimator = IMUPoseEstimator()
    imu_pose_estimator.run()
    imu_pose_estimator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()