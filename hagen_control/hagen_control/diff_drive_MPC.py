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

        self.Ts = 0.033 # Sampling time
        self.n = 0 # Counter for the number of samples
        self.time_utilized = 0
        self.timer = self.create_timer(self.Ts, self.timer_callback)
        self.dTol = 0.08

        self.odom_data = None

        self.end_controller = False
        self.Simulation_q = np.array([0,0,0])
        self.v = []
        self.w = []
        
        t = np.arange(0, 30, self.Ts)
        freq = 2*np.pi/30
        self.xRef = 0.0 + 2.7*np.sin(freq*t)
        self.yRef = 0.0 + 2.7*np.sin(2*freq*t)
        dxRef = freq*2.7*np.cos(freq*t)
        dyRef = 2*freq*2.7*np.cos(2*freq*t)
        ddxRef =-freq**2*2.7*np.sin(freq*t) 
        ddyRef =-4*freq**2*2.7*np.sin(2*freq*t)

        # self.q = np.array([0, 0, 0]) # Initial robot pose
        self.qRef = np.array([self.xRef, self.yRef, np.arctan2(dyRef, dxRef)])
        vRef = np.sqrt(dxRef**2 + dyRef**2)
        wRef = (dxRef*ddyRef - dyRef*ddxRef)/(dxRef**2 + dyRef**2)

        self.uRef = np.array([vRef, wRef]) # Reference inputs
        self.k = 0
        self.h = 4
        B = np.array([[1, 0],
                      [0, 0],
                      [0, 1]])
        self.B = B*self.Ts
        self.C = np.eye(3)
        
        ar = 0.65
        Ar = np.eye(3)*ar # Reference error dynamics 
        H = 0
        self.Fr = np.vstack([Ar**(H+1),Ar**(H+2),Ar**(H+3),Ar**(H+4)])

        # Weight matrices
        self.Q = np.diag(np.full(3*self.h,[0.05, 1.0, 0.1]*self.h)) # [1, 40, 0.1] Penalize X, Y position errors, and YAW ANGLE heading error
        self.R = np.diag(np.full(2*self.h,[0.01, 0.01]*self.h)) # Penalty for linear velocity and angular velocity efforts 


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

    def Ad(self, k):
        # A(k) = I + Ac(t)Ts
        A_k = np.array([[1, self.uRef[1,k]*self.Ts, 0],
                        [-self.uRef[1,k]*self.Ts, 1, self.uRef[0,k]*self.Ts],
                        [0, 0, 1]])
        return A_k


#_____________ reference pose ___________________________________________________________

    def inter_pose_diff_drive(self, ):
        if(self.odom_data is not None):
            print(f'Current State = {self.odom_data}')
            print(f'Desired State = {self.qRef[:,self.k]}')

            # e = np.array([[np.cos(self.q[2]), np.sin(self.q[2]), 0],
            #               [-np.sin(self.q[2]), np.cos(self.q[2]), 0],
            #               [0, 0, 1]])@(self.qRef[:,self.k] - self.q) # Error vector 

            e = np.array([[np.cos(self.phi), np.sin(self.phi), 0],
                          [-np.sin(self.phi), np.cos(self.phi), 0],
                          [0, 0, 1]])@(self.qRef[:,self.k] - self.odom_data) # Error vector 

            e[2] = self.wrap_to_pi(e[2]) # Correct angle 

            A0 = self.Ad(self.k)
            A1 = self.Ad(self.k+1)
            A2 = self.Ad(self.k+2)
            A3 = self.Ad(self.k+3)
            A4 = self.Ad(self.k+4)


            Z = np.zeros((3,2))
            G = np.vstack([np.hstack([self.C@A0@self.B,          Z,                      Z,                   Z]), 
                           np.hstack([self.C@A0@A1@self.B,       self.C@A0@self.B,       Z,                   Z]),
                           np.hstack([self.C@A0@A1@A2@self.B,    self.C@A0@A1@self.B,    self.C@A0@self.B,    Z]),
                           np.hstack([self.C@A0@A1@A2@A3@self.B, self.C@A0@A1@A2@self.B, self.C@A0@A1@self.B, self.C@A0@self.B])])

            F = np.vstack([self.C@A0@A1, self.C@A0@A1@A2, self.C@A0@A1@A2@A3, self.C@A0@A1@A2@A3@A4])
           
           # Optimal generalized predictive control calculation
            KKgpc = np.linalg.inv(G.T@self.Q@G+ self.R)@G.T@self.Q@-F
            KK = KKgpc[0:2,:] # Take current control gains (first 2 rows)

            u_fb = -KK@e
            u_ff = np.array([self.uRef[0,self.k]*np.cos(e[2]), self.uRef[1,self.k]])
            u = u_ff + u_fb

            v = u[0]
            w = u[1]
            if abs(v)>1.0: v=1.0*np.sign(v)
            if abs(w)>np.pi/2: w=np.pi/2*np.sign(w)
            self.send_vel(v, w)
            self.k +=1

            # # Robot motion simulation
            # dq = np.array([u[0]*np.cos(self.q[2]), u[0]*np.sin(self.q[2]), u[1]])
            # self.q = self.q + self.Ts*dq # Euler integration
            # self.q[2] = self.wrap_to_pi(self.q[2]) # Map orientation angle to [-pi, pi]

            self.time_utilized  =  self.time_utilized + self.Ts   
            self.n +=1
            # self.Simulation_q = np.vstack([self.Simulation_q, self.q])
            self.Simulation_q = np.vstack([self.Simulation_q, self.odom_data])
            self.v.append(v)
            self.w.append(w)

            if self.k == self.uRef.shape[1]-self.h:
                print("\nGoal Has Been Reached Successfully!")
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
        plt.subplot(2, 2, 1)
        plt.plot(t, self.v)
        plt.title('velocity control signal', fontsize=15)
        plt.xlabel('time(s)', fontsize=10)
        plt.ylabel('v(m/s)', fontsize=10)

        plt.subplot(2, 2, 2)
        plt.plot(t, self.w)
        plt.title('angular velocity control signal', fontsize=15)
        plt.xlabel('time(s)', fontsize=10)
        plt.ylabel('\u03C9(rad)', fontsize=10)

        plt.subplot(2, 2, 3)
        plt.plot(t, odom[:,0], 'r--', linewidth=2.0, label = r'$x$')
        plt.plot(t, odom[:,1], 'b--', linewidth=2.0, label = r'$y$')
        plt.plot(t, odom[:,2], 'k--', linewidth=2.0, label = r'$\theta$')
        plt.title('robot state', fontsize=15)
        plt.xlabel('time(s)', fontsize=10)
        plt.ylabel(r'Coordinates $x,y,\theta$', fontsize=10)
        plt.legend()

        plt.subplot(2, 2, 4)
        plt.plot(self.xRef, self.yRef, linestyle ="--", color='r', label="Reference trajectory")
        plt.plot(odom[:,0], odom[:,1], label="MPC tracking results")
        plt.scatter(0, 0, color='y', label="Start point")
        plt.title('trajectory tracking using MPC', fontsize=15)
        plt.xlabel('x(m)', fontsize=10)
        plt.ylabel('y(m)', fontsize=10)
        plt.legend(loc="best")
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