import numpy as np 
from matplotlib import pyplot as plt

class PathFollower():
    def __init__(self, delta_t):
        self.i = 0
        self.r = 0.3 # Wheel radius
        self.L = 1.25 # Axle length
        self.D = 0.07 # Distance between the front front whell and rear axle
        self.Ts = delta_t # Sampling time
        self.T = np.arange(0, 40, self.Ts) # Simulation time

        self.q = None
        self.Q_calculated = []
        self.time_utilized = []

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

        sinp = 2.0 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2.0* (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def wrap_to_pi(self, x):
        x = np.array([x])
        xwrap = np.remainder(x, 2.0*np.pi)
        mask = np.abs(xwrap)>np.pi
        xwrap[mask] -= 2.0*np.pi * np.sign(xwrap[mask])
        return xwrap[0]
    
    #define your logic 
    def path_folower(self, pointB, pointC):
        # Statring point
        self.q = [0, 0, np.pi/2]
        r = np.sqrt((self.q[0]-pointB[0])**2 + (self.q[1]-pointB[1])**2)/2
        to_B = True 
        for t in self.T:
            print("q", self.q)
            if to_B:
                D = np.sqrt((self.q[0]-pointB[0])**2 + (self.q[1]-pointB[1])**2)
                print("D", D)
                v = 1
                w = -v/r
                if D < 0.1:
                    to_B = False
            else:
                D = np.sqrt((self.q[0]-pointC[0])**2 + (self.q[1]-pointC[1])**2)
                print("D", D)
                w = 0
                v = 1
                if D < 0.1:
                    print("The goal is reached")
                    break

            # Trapezoidal numerical integration
            dq = np.array([v*np.cos(self.q[2]+self.Ts*w/2),
                        v*np.sin(self.q[2]+self.Ts*w/2), w])
            self.q = self.q + self.Ts*dq
            # Map orientation angle to [-pi, pi]
            self.q[2] = self.wrap_to_pi(self.q[2])
            self.Q_calculated.append(self.q)
            self.time_utilized.append(t)

    
def main(args=None):
    path_folower = PathFollower(delta_t=0.05)
    B, C = [20, 0], [20, -5]
    path_folower.path_folower(B, C)

    Q_calculated = np.array(path_folower.Q_calculated)
    t = path_folower.time_utilized

    fig = plt.figure(figsize=(10, 8))
    
    plt.subplot(2, 2, 1)
    plt.plot(t, Q_calculated[:, 0], label="Calculated X")
    plt.xlabel("time t")
    plt.ylabel("X coord")
    plt.title("X coord")
    plt.legend()

    plt.subplot(2, 2, 2)
    plt.plot(t, Q_calculated[:, 1], label="Calculated Y")
    plt.xlabel("time t")
    plt.ylabel("Y coord")
    plt.title("Y coord")
    plt.legend()

    plt.subplot(2, 2, 3)
    plt.plot(t, Q_calculated[:, 2], label="Calculated Phi")
    plt.xlabel("time t")
    plt.ylabel("Phi angle")
    plt.title("Phi angle")
    plt.legend()

    plt.subplot(2, 2, 4)
    plt.scatter([B[0], C[0]], [B[1], C[1]], color='r')
    plt.plot(Q_calculated[:, 0], Q_calculated[:, 1], label="Calculated X-Y")
    plt.xlabel("X coord")
    plt.ylabel("Y coord")
    plt.title("X-Y coords")
    plt.legend()
    plt.savefig("fig.png")
    plt.show()

if __name__ == '__main__':
    main()
