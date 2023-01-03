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
        self.A = np.eye(3)
        self.R = np.array([[0.01, 0],  # Penalty for linear velocity effort
                           [0, 0.01]]) # Penalty for angular velocity effort

        self.Q = np.array([[0.05, 0, 0],  # Penalize X position error
                           [0, 0.5, 0],   # Penalize Y position error 
                           [0, 0, 0.04]]) # Penalize YAW ANGLE heading error 
        self.max_linear_velocity = 1.0     # meters per second
        self.max_angular_velocity = 1.5708 # radians per second
        self.dTol = 0.08


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

    
    def getB(self):
        """
        Calculates and returns the B matrix 3x2 matix ---> number of states x number of control inputs
        
        Expresses how the state of the system [x,y,yaw] changes from t-1 to t due to the control commands (i.e. control inputs).
        
        :param yaw: The yaw angle (rotation angle around the z axis) in radians 
        :param deltat: The change in time from timestep t-1 to t in seconds
        
        :return: B matrix ---> 3x2 NumPy array
        """
        B = np.array([[np.cos(self.phi)*self.Ts, 0],
                      [np.sin(self.phi)*self.Ts, 0],
                      [0, self.Ts]])
        return B


    def state_space_model(self, B, control_input_t_minus_1):
        state_t_minus_1 = self.odom_data
        """
        Calculates the state at time t given the state at time t-1 and the control inputs applied at time t-1
        
        :param A: The A state transition matrix 3x3 NumPy Array
        :param state_t_minus_1: The state at time t-1 3x1 NumPy Array given the state is [x,y,yaw angle] ---> [meters, meters, radians]
        :param B: The B state transition matrix 3x2 NumPy Array
        :param control_input_t_minus_1: Optimal control inputs at time t-1 2x1 NumPy Array given the control input vector is [linear velocity of the car (m/s), angular velocity of the car (rad/s)]
        
        :return State estimate at time t: 3x1 NumPy Array given the state is [x,y,yaw angle] ---> [meters, meters, radians]
        """

        # These next lines of code which place limits on the angular and linear velocities of the robot car can be removed if you desire.
        control_input_t_minus_1[0] = np.clip(control_input_t_minus_1[0], -self.max_linear_velocity, self.max_linear_velocity)
        control_input_t_minus_1[1] = np.clip(control_input_t_minus_1[1], -self.max_angular_velocity, self.max_angular_velocity)

        state_estimate_t = (self.A @ state_t_minus_1) + (B @ control_input_t_minus_1)

        return state_estimate_t


    def lqr(self, B):
        """
        Discrete-time linear quadratic regulator for a nonlinear system.
        Compute the optimal control inputs given a nonlinear system, cost matrices, current state, and a final state.
        
        Compute the control variables that minimize the cumulative cost.
        
        Solve for P using the dynamic programming method.
        
        :param actual_state_x: The current state of the system 3x1 NumPy Array given the state is [x,y,yaw angle] ---> [meters, meters, radians]
        :param desired_state_xf: The desired state of the system 3x1 NumPy Array given the state is [x,y,yaw angle] ---> [meters, meters, radians]   
        :param Q: The state cost matrix 3x3 NumPy Array
        :param R: The input cost matrix 2x2 NumPy Array
        :param dt: The size of the timestep in seconds -> float
        
        :return u_star: Optimal action u for the current state  2x1 NumPy Array given the control input vector is [linear velocity of the car (m/s), angular velocity of the car (rad/s)]
        """
        # We want the system to stabilize at desired_state_xf.
        x_error = self.odom_data - self.refPose
        
        # Solutions to discrete LQR problems are obtained using the dynamic programming method.
        # The optimal solution is obtained recursively, starting at the last timestep and working backwards.
        # You can play with this number
        N = 50
        # Create a list of N + 1 elements
        P = [None] * (N + 1)
        Qf = self.Q
        
        # LQR via Dynamic Programming
        P[N] = Qf
        
        # For i = N, ..., 1
        for i in range(N, 0, -1):
            # Discrete-time Algebraic Riccati equation to calculate the optimal state cost matrix
            P[i-1] = self.Q + self.A.T @ P[i] @ self.A - (self.A.T @ P[i] @ B) @ np.linalg.pinv(self.R + B.T @ P[i] @ B) @ (B.T @ P[i] @ self.A)      
            
        # Create a list of N elements
        K = [None] * N
        u = [None] * N
            
        # For i = 0, ..., N - 1
        for i in range(N):
            # Calculate the optimal feedback gain K
            K[i] = -np.linalg.pinv(self.R + B.T @ P[i+1] @ B) @ B.T @ P[i+1] @ self.A
            u[i] = K[i] @ x_error
        
        # Optimal control input is u_star
        u_star = u[N-1]
        
        return u_star


#_____________ reference pose ___________________________________________________________

    def inter_pose_diff_drive(self, ):
        if(self.odom_data is not None):
            print(f'Current State = {self.odom_data}')
            print(f'Desired State = {self.refPose}')
            
            state_error = self.odom_data - self.refPose
            state_error_magnitude = np.linalg.norm(state_error)
            print(f'State Error Magnitude = {state_error_magnitude}')

            B = self.getB()

            # LQR returns the optimal control input
            optimal_control_input = self.lqr(B) 
            v = optimal_control_input[0]
            w = optimal_control_input[1]
            if abs(v)>1.0: v=1.0*np.sign(v)
            if abs(w)>np.pi/2: w=np.pi/2*np.sign(w)
            print(f'Control Input = {optimal_control_input}')

            # We apply the optimal control to the robot so we can get a new actual (estimated) state.
            # actual_state_x = self.state_space_model(B, optimal_control_input)  

            self.send_vel(v, w)
            self.time_utilized  =  self.time_utilized + self.Ts   
            self.n +=1
            self.Simulation_q = np.vstack([self.Simulation_q, self.odom_data])
            self.v.append(v)
            self.w.append(w)

            # Stop as soon as we reach the goal
            # Feel free to change this threshold value.
            if state_error_magnitude < self.dTol:
                print("\nGoal Has Been Reached Successfully!")
                self.send_vel(0.0, 0.0)
                self.end_controller = True
                self.t = np.linspace(0, self.time_utilized, self.n) # Create time span
                self.plot(self.Simulation_q[1:], self.t)
            
            print()

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
        plt.grid(color='black', linestyle='--', linewidth=1.0, alpha = 0.7)
        plt.grid(True)
        plt.legend()

        plt.subplot(2, 2, 4)
        plt.plot(odom[:,0], odom[:,1], label="odom")
        plt.scatter(0, 0, color='y', label="Start")
        # plt.plot([odom[0][0], odom[-1][0]], [odom[0][1], odom[-1][1]], linestyle ="--", color='r', label="optimal")
        plt.scatter(self.refPose[0], self.refPose[1], color='k', label="refPose")
        plt.title('odom trajectory', fontsize=15)
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
