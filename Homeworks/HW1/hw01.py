from cProfile import label
from tkinter import font
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.clock import Clock
from rclpy.duration import Duration
import time 
import numpy as np
import matplotlib.pyplot as plt

class MinimalPublisher(Node):
    def __init__(self, delta_t, v, w):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.vel_sub = self.create_subscription(Twist, '/hagen/cmd_vel', self.listener_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, "/hagen/odom", self.set_pose, 20)
        self.publisher_q = self.create_publisher(Twist, '/hagen/q', 20)

        self.end_controller = False
        self.vel_sub
        self.odom_sub
        self.i = 0
        self.set_q_init = None
        self.r = 0.03 # Wheel radius
        self.L = 0.08 # Axle length
        self.D = 0.07 # Distance between the front front whell and rear axle
        self.Ts = delta_t # Sampling time
        self.duration = 5
        self.t = np.arange(0, self.duration, self.Ts) # Simulation time
        self.time_utilized = 0.0
        self.Actual_q = np.array([0,0,0])
        self.Simulation_q = np.array([0,0,0])
        # wL = 20 # Left wheel velocity
        # wR = 18 # Right wheel velocity
        # self.v = self.r/2*(wR+wL) # Robot velocity
        # self.w = self.r/self.L*(wR-wL) # Robot angular velocity

        self.v = v
        self.w = w


    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def listener_callback(self, msg):
        pass    
    
    def wrap_to_pi(self, x):
        x = np.array([x])
        xwrap = np.remainder(x, 2*np.pi)
        mask = np.abs(xwrap)>np.pi
        xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
        return xwrap[0]

    def inter_direction_diff_drive(self, ):
        if self.set_q_init is not None:
            if self.i == 0:
                print(f"Init value is set: {np.round(self.set_q_init,2)}")
                self.q = self.set_q_init # np.array([4 ,0.5, np.pi/6]) # Initial pose
                self.i = 1

            if(self.duration > self.time_utilized):
                self.dq = np.array([self.v*np.cos(self.q[2]+self.Ts*self.w/2), self.v*np.sin(self.q[2]+self.Ts*self.w/2), self.w])
                self.q = self.q + self.Ts*self.dq # Integration
                self.q[2] = self.wrap_to_pi(self.q[2]) # Map orientation angle to [-pi, pi]
                self.publish_q(self.q)

                self.send_vel(self.v, self.w)
                time.sleep(self.Ts)
                self.time_utilized  =  self.time_utilized + self.Ts
                self.Actual_q = np.vstack([self.Actual_q, self.q])
                self.Simulation_q = np.vstack([self.Simulation_q, self.set_q_init])
                print(f'Actual q: {np.round(self.Actual_q[self.i],2)} \t Simulated q: {np.round(self.Simulation_q[self.i],2)}')
                self.i += 1
            
            else:
                print("End of simulation")
                actual = self.Actual_q
                odom = self.Simulation_q
                self.send_vel(0.0, 0.0)
                self.plot(actual[1:], odom[1:])
                self.end_controller = True


    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.set_q_init = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
        self.inter_direction_diff_drive()


    def publish_q(self, q):
        msg = Twist()
        msg.linear.x = q[0]
        msg.angular.z = q[2]
        self.publisher_q.publish(msg)

    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_.publish(msg)


    def plot(self, actual, odom):
        plt.plot(self.t, actual[:,0], label="q")
        plt.plot(self.t, odom[:,0], label="odom")
        plt.title('x analytical vs corresponding odom x', fontsize=15)
        plt.xlabel('time(s)', fontsize=15)
        plt.ylabel('x(m)', fontsize=15)
        plt.legend(loc="lower right")
        plt.show()

        plt.plot(self.t, actual[:,1], label="q")
        plt.plot(self.t, odom[:,1], label="odom")
        plt.title('y analytical vs corresponding odom y', fontsize=15)
        plt.xlabel('time(s)', fontsize=15)
        plt.ylabel('y(m)', fontsize=15)
        plt.legend(loc="lower right")
        plt.show()

        plt.plot(self.t, actual[:,2], label="q")
        plt.plot(self.t, odom[:,2], label="odom")
        plt.title('yaw analytical vs corresponding odom yaw', fontsize=15)
        plt.xlabel('time(s)', fontsize=15)
        plt.ylabel('yaw', fontsize=15)
        plt.legend(loc="lower right")
        plt.show()
    

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher(delta_t=0.033, v=0.5, w=0.0)
    
    while minimal_publisher.end_controller is False and rclpy.ok():
        try:
            rclpy.spin_once(minimal_publisher)

        except KeyboardInterrupt:
            break
            
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()