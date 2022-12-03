import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np 
import matplotlib.pyplot as plt

class MinimalPublisher(Node):
    def __init__(self, delta_t):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/hagen/cmd_vel', 30)
        self.odom_sub = self.create_subscription(Odometry, "/hagen/odom", self.set_pose, 20)
        self.odom_sub

        self.set_q_init = None
        self.q = None 
        self.odom_data = None

        self.Ts = delta_t # Sampling time
        self.end_controller = False
        self.timer = self.create_timer(self.Ts, self.timer_callback)
        self.Actual_q = np.array([0,0,0])
        self.Simulation_q = np.array([0,0,0])
        self.n = 0
        self.path_plot = 0
    
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

    def inter_pose_diff_drive_init(self, duration=30, refPose=np.array([3,2,0]), k_p=0.8, k_w=5, dTol=0.03):
        self.duration = duration
        self.refPose = refPose
        self.k_p = k_p 
        self.k_w = k_w
        self.dTol = dTol
        self.time_utilized = 0.0 

    def inter_pose_diff_drive(self, ):
        if(self.q is not None):
            self.D = np.sqrt((self.q[0]-self.refPose[0])**2 + (self.q[1]-self.refPose[1])**2)
            print("Distance to the goal: ", self.D)
            phiR = np.arctan2(self.refPose[1]-self.q[1], self.refPose[0]-self.q[0])
            ePhi = self.wrap_to_pi(phiR - self.q[2])
            
            v = self.k_p*self.D
            w = self.k_w*ePhi
            
            if abs(v)>0.8: v=0.8*np.sign(v)
            if abs(w)>np.pi/4: w=np.pi/4*np.sign(w)

            # Trapz Integration
            # dq = np.array([v*np.cos(self.q[2]+self.Ts*w/, v*np.sin(self.q[2]+self.Ts*w/2), w])
            # self.q = self.q + self.Ts*dq 

            # Euler Integration 
            dq = np.array([v*np.cos(self.q[2]), v*np.sin(self.q[2]), w])
            self.q = self.q + self.Ts*dq 
            self.q[2] = self.wrap_to_pi(self.q[2]) # Map orientation angle to [-pi, pi]
            self.send_vel(v, w)
            self.time_utilized  =  self.time_utilized + self.Ts   
            self.n +=1
            self.Actual_q = np.vstack([self.Actual_q, self.q])
            self.Simulation_q = np.vstack([self.Simulation_q, self.odom_data])

            if(self.D < self.dTol):
                print("Reach to the goal pose")
                print("q data: ",self.q)
                print("Odom data: ", self.odom_data)
                self.send_vel(0.0, 0.0)
                self.end_controller = True
                self.t = np.linspace(0, self.time_utilized, self.n) # Create time span
                self.plot(self.Actual_q[1:], self.Simulation_q[1:], self.t)

            if(self.duration < self.time_utilized):
                print("q data: ",self.q)
                print("Odom data: ", self.odom_data)
                print("End of simulation")
                self.send_vel(0.0, 0.0)
                self.end_controller = True
    
#_____________ intermediate point ___________________________________________________________

    def inter_point_diff_drive_init(self, duration=30, r_distance=1.3
                    , refPose=np.array([3,6,0]), k_p=0.5, k_w=5, dTol=0.05):
        self.duration = duration
        self.r_distance = r_distance
        self.refPose = refPose
        self.k_p = k_p 
        self.k_w = k_w
        self.xT = refPose[0]-r_distance*np.cos(refPose[2])
        self.yT = refPose[1]-r_distance*np.sin(refPose[2])
        self.dTol = dTol
        self.time_utilized = 0.0 
        self.state = 0

    def inter_point_diff_drive(self, ):
        if(self.q is not None):
            if(self.duration < self.time_utilized):
                print("End of simulation")
                self.send_vel(0.0, 0.0)
                self.end_controller = True

            self.D = np.sqrt((self.q[0]-self.refPose[0])**2 
                                    + (self.q[1]-self.refPose[1])**2)

            if(self.D < self.dTol):
                print("Reach to the goal pose")
                print("q data: ",self.q)
                print("Odom data: ", self.odom_data)

                self.send_vel(0.0, 0.0)
                self.end_controller = True
            
            else:
                if (self.state == 0):
                    d = np.sqrt((self.q[0]-self.xT)**2 + (self.q[1]-self.yT)**2)
                    print("Distance to the midpoint: ", d)
                    phiT = np.arctan2(self.yT-self.q[1], self.xT-self.q[0])
                    ePhi = self.wrap_to_pi(phiT - self.q[2])
                    if (d < self.dTol): 
                        self.state = 1
                        print("Reach to the mid pose")
                        print("q data: ",self.q)
                        print("Odom data: ", self.odom_data)
                
                else:
                    print("Distance to the goal: ", self.D)
                    phiR = np.arctan2(self.refPose[1]-self.q[1], self.refPose[0]-self.q[0])
                    ePhi = self.wrap_to_pi(phiR - self.q[2])
                     
                v = self.k_p*self.D
                w = self.k_w*ePhi
                dq = np.array([v*np.cos(self.q[2]+self.Ts*w/2)
                                , v*np.sin(self.q[2]+self.Ts*w/2), w])
                self.q = self.q + self.Ts*dq # Integration
                self.q[2] = self.wrap_to_pi(self.q[2]) # Map orientation angle to [-pi, pi]
                self.send_vel(v, w)
                self.time_utilized  =  self.time_utilized + self.Ts   

#_____________ intermediate direction ___________________________________________________________

    def inter_direction_diff_drive_init(self, duration=30, r_distance=1.3
                    , refPose=np.array([3,6,0]), k_p=0.5, k_w=0.7, dTol=0.7):
        self.duration = duration
        self.r_distance = r_distance
        self.refPose = refPose
        self.k_p = k_p 
        self.k_w = k_w
        self.dTol = dTol
        self.time_utilized = 0.0 

    def inter_direction_diff_drive(self, ):
        if(self.q is not None):
            if(self.duration < self.time_utilized):
                print("End of simulation")
                self.send_vel(0.0, 0.0)
                self.end_controller = True

            self.D = np.sqrt((self.q[0]-self.refPose[0])**2 
                                    + (self.q[1]-self.refPose[1])**2)

            if(self.D < self.dTol):
                print("Reach to the goal pose")
                self.send_vel(0.0, 0.0)
                self.end_controller = True

                
            beta = np.arctan(self.r_distance/self.D)
            phiR = np.arctan2(self.refPose[1]-self.q[1], self.refPose[0]-self.q[0])
            alpha = self.wrap_to_pi(phiR-self.refPose[2])
            if(alpha <0):
                beta = -beta
            ##Controller
            if(np.abs(alpha) < np.abs(beta)):
                ePhi = self.wrap_to_pi(phiR - self.q[2] + alpha) 
            else:
                ePhi = self.wrap_to_pi(phiR - self.q[2] + beta) 
            v = self.k_p*self.D
            w = self.k_w*ePhi
            print("Distance to the goal: ", self.D)
            dq = np.array([v*np.cos(self.q[2]+self.Ts*w/2)
                            , v*np.sin(self.q[2]+self.Ts*w/2), w])
            self.q = self.q + self.Ts*dq # Integration
            self.q[2] = self.wrap_to_pi(self.q[2]) # Map orientation angle to [-pi, pi]
            self.send_vel(v, w)
            self.time_utilized  =  self.time_utilized + self.Ts   

#_____________ path control ___________________________________________________________

    def inter_path_diff_drive_init(self, duration=50, r_distance=1.3
                    , refPoints=np.array([[0,0],[3,0],[6,4],[3,4],[3,1],[0,3]]), k_p=0.4, k_w=3, dTol=0.05):
        self.duration = duration
        self.r_distance = r_distance
        self.T = refPoints
        self.k_p = k_p 
        self.k_w = k_w
        self.dTol = dTol
        self.time_utilized = 0.0 
        self.i = 0
        self.path_plot = 1

    def inter_path_diff_drive(self, ):
        if(self.q is not None):
            if(self.duration < self.time_utilized):
                print("End of simulation")
                self.send_vel(0.0, 0.0)
                self.end_controller = True

            self.D = np.sqrt((self.q[0]-self.T[-1,0])**2 
                                    + (self.q[1]-self.T[-1,1])**2)
            if(self.D < self.dTol):
                print("reached point: ", self.T[self.i+1])
                print("q data: ", np.round(self.q,2))
                print("odom data: ", np.round(self.odom_data,2))
                print("--------------------------------------------", "\n")
                print("Reach to the final point")
                self.send_vel(0.0, 0.0)
                self.end_controller = True
                self.t = np.arange(0, self.time_utilized-self.Ts , self.Ts) # Simulation time
                self.plot(self.Actual_q[1:], self.Simulation_q[1:], self.t)

            dx = self.T[self.i+1, 0] - self.T[self.i, 0]
            dy = self.T[self.i+1, 1] - self.T[self.i, 1]

            v = np.array([dx,dy]) # Direction vector of the current segment
            vN = np.array([dy,-dx]) #  Orthogonal direction vector of the current segment
            r = self.q[:2] - self.T[self.i]
            u = v.T.dot(r)/(v.T.dot(v)) 

            
            if u > 1 and self.i < len(self.T): # Switch to the next line segment
      
                self.i += 1
                print("reached point: ", self.T[self.i])
                print("q data: ", np.round(self.q,2))
                print("odom data: ", np.round(self.odom_data,2))
                print("--------------------------------------------", "\n")

                dx = self.T[self.i+1, 0] - self.T[self.i, 0]
                dy = self.T[self.i+1, 1] - self.T[self.i, 1]
                v = np.array([dx,dy]) 
                vN = np.array([dy,-dx]) 
                r = self.q[:2] - self.T[self.i]

            dn = vN.T.dot(r)/(vN.T.dot(vN)) # Normalized orthogonal distance

            phiLin = np.arctan2(v[1], v[0]) # Orientation of the line segm 
            phiRot = np.arctan(5*dn); # If we are far from the line then we need additional rotation to face towards the line. 
            # If we are on the left side of the line we turn clockwise, otherwise counterclock wise.
            # Gain 5 increases the sensitivity ...
            phiRef = self.wrap_to_pi(phiLin + phiRot)
            ePhi = self.wrap_to_pi(phiRef - self.q[2]) # Orientation error for control

            v = self.k_p*np.cos(ePhi)
            w = self.k_w*ePhi

            dq = np.array([v*np.cos(self.q[2]+self.Ts*w/2)
                            , v*np.sin(self.q[2]+self.Ts*w/2), w])
            self.q = self.q+ self.Ts*dq # Integration
            self.q[2] = self.wrap_to_pi (self.q[2]) # Map orientation angle to [-pi, pi]
            self.send_vel(v, w)
            self.time_utilized  =  self.time_utilized + self.Ts   
            self.Actual_q = np.vstack([self.Actual_q, self.q])
            self.Simulation_q = np.vstack([self.Simulation_q, self.odom_data])

    def timer_callback(self, ):
        if self.end_controller is False:
            # self.inter_pose_diff_drive()
            # self.inter_point_diff_drive()
            # self.inter_direction_diff_drive()
            self.inter_path_diff_drive()
        else:
            self.destroy_timer(self.timer)
            return 
            
    def set_pose(self, msg):
        _, _, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)
        if(self.set_q_init is None):
            self.set_q_init = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])
            self.q = self.odom_data = self.set_q_init
        else:
            self.odom_data = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, yaw])


    def send_vel(self, v, w):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.publisher_.publish(msg)

    
    def plot(self, actual_q, odom, t):
        fig = plt.figure(figsize=(10, 8))
        plt.subplot(2, 2, 1)
        plt.plot(t, actual_q[:,0], label="X")
        plt.plot(t, odom[:,1], label="odom")
        plt.title('x analytical vs corresponding odom x', fontsize=10)
        plt.xlabel('time(s)', fontsize=10)
        plt.ylabel('x(m)', fontsize=10)
        plt.legend(loc="lower right")

        plt.subplot(2, 2, 2)
        plt.plot(t, actual_q[:,1], label="Y")
        plt.plot(t, odom[:,1], label="odom")
        plt.title('y analytical vs corresponding odom y', fontsize=10)
        plt.xlabel('time(s)', fontsize=10)
        plt.ylabel('y(m)', fontsize=10)
        plt.legend(loc="lower right")

        plt.subplot(2, 2, 3)
        plt.plot(actual_q[:,0], actual_q[:,1], label="X-Y") 
        if self.path_plot == 0:
            plt.scatter(0, 0, color='r', label="Start")
            plt.scatter(self.refPose[0], self.refPose[1], color='k', label="refPose")
        else: 
            plt.plot(self.T[1:,0], self.T[1:,1], linestyle ="--", color='r',label="refPose")
            plt.scatter(self.T[1:,0], self.T[1:,1], color='k')        
        plt.title('X-Y trajectory', fontsize=10)
        plt.xlabel('x(m)', fontsize=10)
        plt.ylabel('y(m)', fontsize=10)
        plt.legend(loc="lower right")

        plt.subplot(2, 2, 4)
        plt.plot(actual_q[:,0], actual_q[:,1], label="q")
        plt.plot(odom[:,0], odom[:,1], label="odom")
        plt.title('q vs odom trajectory', fontsize=10)
        plt.xlabel('x(m)', fontsize=10)
        plt.ylabel('y(m)', fontsize=10)
        plt.legend(loc="lower right")
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher(delta_t=0.03)
    # minimal_publisher.inter_pose_diff_drive_init()
    # minimal_publisher.inter_point_diff_drive_init()
    # minimal_publisher.inter_direction_diff_drive_init()
    minimal_publisher.inter_path_diff_drive_init()

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
