import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np 
import matplotlib.pyplot as plt

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.odom_sub = self.create_subscription(Odometry, "/hagen/odom", self.set_pose, 20)
        self.odom_sub

        self.Ts = 0.03 # Sampling time
        self.n = 0 # Counter for the number of samples
        self.time_utilized = 0
        self.timer = self.create_timer(self.Ts, self.timer_callback)

        self.odom_data = None

        self.end_controller = False
        self.Simulation_q = np.array([0,0,0])
        self.v = []
        self.w = []
        self.refPose = np.array([5,5,np.pi/2])
        self.kp = 3 
        self.kd = 0.05
        self.dTol = 0.05
        self.position = 0
    
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


    def wrap_to_pi(self, x):
        x = np.array([x])
        xwrap = np.remainder(x, 2*np.pi)
        mask = np.abs(xwrap)>np.pi
        xwrap[mask] -= 2*np.pi * np.sign(xwrap[mask])
        return xwrap[0]

#_____________ reference pose ___________________________________________________________

    def inter_pose_diff_drive(self, ):
        if(self.odom_data is not None):
            if (self.position == 0):
                self.D = np.sqrt((self.x-self.refPose[0])**2 + (self.y-self.refPose[1])**2)
                print("Distance to the goal: ", self.D)
                
                phiR = np.arctan2(self.refPose[1]-self.y, self.refPose[0]-self.x)
                ePhi = self.wrap_to_pi(phiR - self.phi)
                phiR_dot = (self.y_dot*(self.x-self.refPose[0])-self.x_dot*(self.y-self.refPose[1]))/((self.x-self.refPose[0])**2+(self.y-self.refPose[1])**2)
                ePhi_dot = self.wrap_to_pi(phiR_dot - self.phi_dot)
                v = self.kp*self.D
                w = self.kp*ePhi + self.kd*ePhi_dot
                
                if abs(v)>0.8: v=0.8*np.sign(v)
                if abs(w)>np.pi/4: w=np.pi/4*np.sign(w)


                self.send_vel(v, w)
                self.time_utilized  =  self.time_utilized + self.Ts   
                self.n +=1
                self.Simulation_q = np.vstack([self.Simulation_q, self.odom_data])
                self.v.append(v)
                self.w.append(w)

                if(self.D < self.dTol):
                    print("Reach to the goal pose")
                    print("Odom data: ", self.odom_data)
                    self.send_vel(0.0, 0.0)
                    self.position = 1
                    # self.end_controller = True
                    # self.t = np.linspace(0, self.time_utilized, self.n) # Create time span
                    # self.plot(self.Simulation_q[1:], self.t)

            else:
                ePhi = self.wrap_to_pi(self.refPose[2] - self.phi)
                ePhi_dot = self.wrap_to_pi(-self.phi_dot)
                v = 0.0
                w = (self.kp/10)*ePhi + (self.kd/10)*ePhi_dot
                
                if abs(w)>np.pi/4: w=np.pi/4*np.sign(w)

                self.send_vel(v, w)
                self.time_utilized  =  self.time_utilized + self.Ts   
                self.n +=1
                self.Simulation_q = np.vstack([self.Simulation_q, self.odom_data])
                self.v.append(v)
                self.w.append(w)
                if abs(ePhi) < self.dTol:
                    print("correct the orientation")
                    print("Odom data: ", self.odom_data)
                    self.send_vel(0.0, 0.0)
                    self.end_controller = True
                    self.t = np.linspace(0, self.time_utilized, self.n) # Create time span
                    self.plot(self.Simulation_q[1:], self.t)

#________________________________________________________________________________________

    def timer_callback(self, ):
        if self.end_controller is False:
            self.inter_pose_diff_drive()
        else:
            self.destroy_timer(self.timer)
            return 
            

    def set_pose(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        _, _, self.phi = self.euler_from_quaternion(msg.pose.pose.orientation)
        self.x_dot = msg.twist.twist.linear.x
        self.y_dot = msg.twist.twist.linear.y
        self.phi_dot = msg.twist.twist.angular.z   

        self.odom_data  = np.array([self.x, self.y, self.phi])


    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_.publish(msg)

    
    def plot(self, odom, t):
        fig = plt.figure(figsize=(20, 5))
        plt.subplot(1, 3, 1)
        plt.plot(t, self.v)
        plt.title('velocity control signal', fontsize=10)
        plt.xlabel('time(s)', fontsize=10)
        plt.ylabel('v(m/s)', fontsize=10)

        plt.subplot(1, 3, 2)
        plt.plot(t, self.w)
        plt.title('angular velocity control signal', fontsize=10)
        plt.xlabel('time(s)', fontsize=10)
        plt.ylabel('\u03C9(rad)', fontsize=10)

        plt.subplot(1, 3, 3)
        plt.plot(odom[:,0], odom[:,1], label="odom")
        plt.scatter(0, 0, color='y', label="Start")
        plt.plot([odom[0][0], odom[-1][0]], [odom[0][1], odom[-1][1]], linestyle ="--", color='r', label="optimal")
        plt.scatter(self.refPose[0], self.refPose[1], color='k', label="refPose")
        plt.title('odom trajectory', fontsize=10)
        plt.xlabel('x(m)', fontsize=10)
        plt.ylabel('y(m)', fontsize=10)
        plt.legend(loc="lower right")
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    while minimal_publisher.end_controller is False and rclpy.ok():
        try:
            rclpy.spin_once(minimal_publisher)
        except KeyboardInterrupt:
            break
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

# To reset ros2 service call /reset_simulation std_srvs/srv/Empty
