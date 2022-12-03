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
    
    def path_folower(self, ):
        self.q = np.array([0, 0, np.pi/2])
        B = np.array([20, 0, -np.pi/2])
        C = np.array([20, -5, -np.pi/2])

        # Calculate arc path (half circle)
        r = np.sqrt((self.q[0]-B[0])**2 + (self.q[1]-B[1])**2)/2
        alpha = np.linspace(np.pi, 0, 181)
        x = r*np.cos(alpha)+r
        y = r*np.sin(alpha)

        for i in range(len(alpha)):
            # Calculate PhiR
            if x[i] > r:
                phiR = np.arctan2(y[i],x[i])-np.pi/2
            elif x[i] == y[i]:
                phiR = 0
            elif x[i] < r:
                phiR = np.arctan2(y[i],x[i])

            refPose = np.array([x[i], y[i], phiR])
   
            D = np.sqrt((self.q[0]-refPose[0])**2 + (self.q[1]-refPose[1])**2)
            ePhi = self.wrap_to_pi(phiR - self.q[2])
            v = 10*D
            w = -v/r
            dq = np.array([v*np.cos(self.q[2]+self.Ts*w/2) , v*np.sin(self.q[2]+self.Ts*w/2), w])
            self.q = self.q + self.Ts*dq
            self.q[2] = self.wrap_to_pi(self.q[2])
            self.Q_calculated.append(self.q)
            print(self.q)

def main(args=None):
    path_folower = PathFollower(delta_t=0.05)
    path_folower.path_folower()
    Q_calculated = np.array(path_folower.Q_calculated)
    
    plt.plot(Q_calculated[:, 0], Q_calculated[:, 1], label="X-Y")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("X-Y")
    plt.legend()
    plt.savefig("fig.png")
    plt.show()

if __name__ == '__main__':
    main()