#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
from tf_transformations import euler_from_quaternion

class ComplementaryFilter(Node):
    def __init__(self):
        super().__init__('imu_complementary_filter')

        self.initialized = False
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0
        self.wx_bias, self.wy_bias, self.wz_bias = 0.0, 0.0, 0.0
        self.wx_prev, self.wy_prev, self.wz_prev = 0.0, 0.0, 0.0
        self.gain_acc = 0.01
        self.bias_alpha = 0.01
        self.kGravity = 9.81
        self.kAccelerationThreshold = 0.1
        self.kDeltaAngularVelocityThreshold = 0.01
        self.kAngularVelocityThreshold = 0.2
        self.do_bias_estimation = True
        self.do_adaptive_gain = False
        self.dt = 0.01

        self.filtered_imu = Imu()

        self.prev_time = None

        self.pose_pub = self.create_publisher(PoseStamped, '/imu/tools/com_non_adaptive/pose', 5)
        self.filtered_imu_pub = self.create_publisher(Imu, '/imu/filtered_com', 5)

        self.imu_sub = self.create_subscription(Imu, '/sensors/imu', self.imu_callback, 5)

    def update(self, ax, ay, az, wx, wy, wz, dt):
        if not self.initialized:
            self.get_measurement(ax, ay, az)
            self.initialized = True
            return

        if self.do_bias_estimation:
            self.update_biases(ax, ay, az, wx, wy, wz)

        q0_pred, q1_pred, q2_pred, q3_pred = self.get_prediction(wx, wy, wz, dt)

        dq0_acc, dq1_acc, dq2_acc, dq3_acc = self.get_acc_correction(ax, ay, az, q0_pred, q1_pred, q2_pred, q3_pred)

        gain = self.gain_acc
        if self.do_adaptive_gain:
            gain = self.get_adaptive_gain(self.gain_acc, ax, ay, az)
        else:
            gain = self.gain_acc

        dq0_acc, dq1_acc, dq2_acc, dq3_acc = self.scale_quaternion(gain, dq0_acc, dq1_acc, dq2_acc, dq3_acc)

        self.quaternion_multiplication(q0_pred, q1_pred, q2_pred, q3_pred, dq0_acc, dq1_acc, dq2_acc, dq3_acc)

        self.q0, self.q1, self.q2, self.q3 = self.normalize_quaternion(self.q0, self.q1, self.q2, self.q3)

        self.publish_pose()

    def get_measurement(self, ax, ay, az):
        ax, ay, az = self.normalize_vector(ax, ay, az)

        if az >= 0:
            self.q0 = math.sqrt((az + 1) * 0.5)
            self.q1 = -ay / (2.0 * self.q0)
            self.q2 = ax / (2.0 * self.q0)
            self.q3 = 0.0
        else:
            X = math.sqrt((1 - az) * 0.5)
            self.q0 = -ay / (2.0 * X)
            self.q1 = X
            self.q2 = 0.0
            self.q3 = ax / (2.0 * X)

    def normalize_vector(self, x, y, z):
        norm = math.sqrt(x * x + y * y + z * z)
        x /= norm
        y /= norm
        z /= norm

        return x, y, z

    def update_biases(self, ax, ay, az, wx, wy, wz):
        steady_state = self.check_state(ax, ay, az, wx, wy, wz)

        if steady_state:
            self.wx_bias += self.bias_alpha * (wx - self.wx_bias)
            self.wy_bias += self.bias_alpha * (wy - self.wy_bias)
            self.wz_bias += self.bias_alpha * (wz - self.wz_bias)
        else:
            self.wx_prev, self.wy_prev, self.wz_prev = wx, wy, wz

    def check_state(self, ax, ay, az, wx, wy, wz):
        acc_magnitude = math.sqrt(ax * ax + ay * ay + az * az)
        if abs(acc_magnitude - self.kGravity) > self.kAccelerationThreshold:
            return False

        if abs(wx - self.wx_prev) > self.kDeltaAngularVelocityThreshold or \
           abs(wy - self.wy_prev) > self.kDeltaAngularVelocityThreshold or \
           abs(wz - self.wz_prev) > self.kDeltaAngularVelocityThreshold:
            return False

        if abs(wx - self.wx_bias) > self.kAngularVelocityThreshold or \
           abs(wy - self.wy_bias) > self.kAngularVelocityThreshold or \
           abs(wz - self.wz_bias) > self.kAngularVelocityThreshold:
            return False

        return True

    def get_prediction(self, wx, wy, wz, dt):
        wx_unb = wx - self.wx_bias
        wy_unb = wy - self.wy_bias
        wz_unb = wz - self.wz_bias

        q0_pred = self.q0 + 0.5 * dt * (wx_unb * self.q1 + wy_unb * self.q2 + wz_unb * self.q3)
        q1_pred = self.q1 + 0.5 * dt * (-wx_unb * self.q0 - wy_unb * self.q3 + wz_unb * self.q2)
        q2_pred = self.q2 + 0.5 * dt * (wx_unb * self.q3 - wy_unb * self.q0 - wz_unb * self.q1)
        q3_pred = self.q3 + 0.5 * dt * (-wx_unb * self.q2 + wy_unb * self.q1 - wz_unb * self.q0)

        q0_pred, q1_pred, q2_pred, q3_pred = self.normalize_quaternion(q0_pred, q1_pred, q2_pred, q3_pred)
        return q0_pred, q1_pred, q2_pred, q3_pred

    def get_acc_correction(self, ax, ay, az, p0, p1, p2, p3):
        ax, ay, az = self.normalize_vector(ax, ay, az)
        gx, gy, gz = self.rotate_vector_by_quaternion(ax, ay, az, p0, -p1, -p2, -p3)

        dq0 = math.sqrt((gz + 1) * 0.5)
        dq1 = -gy / (2.0 * dq0)
        dq2 = gx / (2.0 * dq0)
        dq3 = 0.0

        return dq0, dq1, dq2, dq3

    def rotate_vector_by_quaternion(self, x, y, z, q0, q1, q2, q3):
        vx = (q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * x + 2 * (q1 * q2 - q0 * q3) * y + 2 * (q1 * q3 + q0 * q2) * z
        vy = 2 * (q1 * q2 + q0 * q3) * x + (q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3) * y + 2 * (q2 * q3 - q0 * q1) * z
        vz = 2 * (q1 * q3 - q0 * q2) * x + 2 * (q2 * q3 + q0 * q1) * y + (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * z
        return vx, vy, vz

    def get_adaptive_gain(self, alpha, ax, ay, az):
        a_mag = math.sqrt(ax * ax + ay * ay + az * az)

        error = abs(a_mag - self.kGravity) / self.kGravity

        error1, error2 = 0.1, 0.2

        m = 1.0 / (error1 - error2)

        b = 1.0 - m * error1

        factor = 1

        if error < error1:
            factor = 1
        elif error < error2:
            factor = m * error + b
        else:
            factor = 0.0

        return factor * alpha

    def scale_quaternion(self, gain, dq0, dq1, dq2, dq3):
        if dq0 < 0.0:
            # Slerp (Spherical linear interpolation):
            angle = math.acos(dq0)
            A = math.sin(angle * (1.0 - gain)) / math.sin(angle)
            B = math.sin(angle * gain) / math.sin(angle)
            dq0 = A + B * dq0
            dq1 = B * dq1
            dq2 = B * dq2
            dq3 = B * dq3
        else:
            # Lerp (Linear interpolation):
            dq0 = (1.0 - gain) + gain * dq0
            dq1 = gain * dq1
            dq2 = gain * dq2
            dq3 = gain * dq3

        dq0, dq1, dq2, dq3 = self.normalize_quaternion(dq0, dq1, dq2, dq3)

        return dq0, dq1, dq2, dq3

    def quaternion_multiplication(self, p0, p1, p2, p3, q0, q1, q2, q3):
        self.q0 = p0 * q0 - p1 * q1 - p2 * q2 - p3 * q3
        self.q1 = p0 * q1 + p1 * q0 + p2 * q3 - p3 * q2
        self.q2 = p0 * q2 - p1 * q3 + p2 * q0 + p3 * q1
        self.q3 = p0 * q3 + p1 * q2 - p2 * q1 + p3 * q0

    def normalize_quaternion(self, q0, q1, q2, q3):
        norm = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)

        self.q0 = q0 / norm
        self.q1 = q1 / norm
        self.q2 = q2 / norm
        self.q3 = q3 / norm
        return self.q0, self.q1, self.q2, self.q3
    
    def invert_quaternion(self,q0,q1,q2,q3):
        
        q0_inv=q0
        q1_inv=-q1
        q2_inv=-q2
        q3_inv=-q3
        
        return q0_inv,q1_inv,q2_inv,q3_inv
        

    def publish_pose(self):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "imu_link"
        
        q0_inv,q1_inv,q2_inv,q3_inv=self.invert_quaternion(self.q0,self.q1,self.q2,self.q3)

        pose_msg.pose.orientation.w = q0_inv
        pose_msg.pose.orientation.x = q1_inv
        pose_msg.pose.orientation.y = q2_inv
        pose_msg.pose.orientation.z = q3_inv

        quaternion = [pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w]
      
        euler = euler_from_quaternion(quaternion)
        roll, pitch, yaw = euler
        # self.get_logger().info(f"Updated Euler Angles: yaw: {yaw*(180/np.pi):.4f}, pitch: {pitch*(180/np.pi):.4f}, roll: {roll*(180/np.pi):.4f}")

        self.pose_pub.publish(pose_msg)

        self.filtered_imu.angular_velocity.x -= self.wx_bias
        self.filtered_imu.angular_velocity.y -= self.wy_bias
        self.filtered_imu.angular_velocity.z -= self.wz_bias

        self.filtered_imu.orientation.x = pose_msg.pose.orientation.x
        self.filtered_imu.orientation.y = pose_msg.pose.orientation.y
        self.filtered_imu.orientation.z = pose_msg.pose.orientation.z
        self.filtered_imu.orientation.w = pose_msg.pose.orientation.w

        self.filtered_imu_pub.publish(self.filtered_imu)

    def imu_callback(self, data):
        self.filtered_imu = data
        ax, ay, az = data.linear_acceleration.x, data.linear_acceleration.y, data.linear_acceleration.z
        wx, wy, wz = data.angular_velocity.x, data.angular_velocity.y, data.angular_velocity.z

        self.update(ax, ay, az, wx, wy, wz, self.dt)

def main(args=None):
    rclpy.init(args=args)
    filter = ComplementaryFilter()
    rclpy.spin(filter)
    filter.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()