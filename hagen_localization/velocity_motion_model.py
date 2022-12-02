import numpy as np
import matplotlib.pyplot as plt


x0 = np.array([0, 0, -np.pi/10])  # initial pose
dt = 0.05  # time step
v = 2.4  # linear velocity
w = (np.pi/5)  # angular velocity
n = 6  # number of samples
u = np.array([v, w])  # the velocity command


def plot_data(model_data, ax, color_):
    # calculate position and direction vectors:
    size_of_points = len(model_data[:, 0])
    x0 = model_data[:, 0][range(size_of_points-1)]
    x1 = model_data[:, 0][range(1, size_of_points)]
    y0 = model_data[:, 1][range(size_of_points-1)]
    y1 = model_data[:, 1][range(1, size_of_points)]
    xpos = (x0+x1)/2
    ypos = (y0+y1)/2
    xdir = x1-x0
    ydir = y1-y0
    # plot arrow on each line:
    for X, Y, dX, dY in zip(xpos, ypos, xdir, ydir):
        ax.annotate("", xytext=(X, Y), xy=(X+0.001*dX, Y+0.001*dY),
                    arrowprops=dict(arrowstyle="->", color=color_), size=20)


def sample_normal_distribution(b):
    # sample normal distribution with 0 mean, variance b
    rand = np.random.uniform(low=-1.0, high=1.0, size=12)
    return b * np.sum(rand) / 6

# ideal motion model
def velocity_motion_model_ideal(xk, u, dt):
    v = u[0]
    w = u[1]
    x = xk[0]
    y = xk[1]
    theta = xk[2]

    xp = x - (v/w)*np.sin(theta) + (v/w)*np.sin(theta + w*dt)
    yp = y + (v/w)*np.cos(theta) - (v/w)*np.cos(theta + w*dt)
    thetap = theta + w*dt

    x = np.array([xp, yp, thetap])
    return x

# noisy motion model
def velocity_motion_model_noisy(xk, u, dt, noise):
    v = u[0]
    w = u[1]
    x = xk[0]
    y = xk[1]
    theta = xk[2]

    vp = v + noise[0]
    wp = w + noise[1]

    xp = x - (vp/wp)*np.sin(theta) + (vp/wp)*np.sin(theta + wp*dt)
    yp = y + (vp/wp)*np.cos(theta) - (vp/wp)*np.cos(theta + wp*dt)
    thetap = theta + wp*dt + noise[2]*dt

    x = np.array([xp, yp, thetap])
    return x

# bicycle's velocity motion model
def bicycle_motion_model(x0, u, dt, sigma_v, sigma_alpha, ):
    l = 0.1
    v, alpha = u
    x, y, theta = x0
    v_hat = v + sample_normal_distribution(sigma_v**2)
    alpha_hat = alpha + sample_normal_distribution(sigma_alpha**2)
    # generate sample
    x_p = x + v_hat * dt * (np.cos(theta) - np.tan(alpha_hat) * np.sin(theta))
    y_p = y + v_hat * dt * (np.sin(theta) + np.tan(alpha_hat) * np.cos(theta))
    theta_p = theta + v_hat * dt * np.tan(alpha_hat) / l

    return np.array([x_p, y_p, theta_p])


# plotting the motion model
ideal_model = []
noisy_model = []
v_var = 2.5
omega_var = 4.7
theta_car = 0.6
# noise = [np.random.normal(0, v_var), np.random.normal(0, omega_var), np.random.normal(0, theta_car)]
noise = [1.5, 2.4, 0.7]
x_ideal_i = velocity_motion_model_noisy(x0, u, dt, noise)
x_noise_i = velocity_motion_model_ideal(x0, u, dt)
ideal_model.append(x0)
noisy_model.append(x0)
for i in range(0, n):
    x_ideal_i = velocity_motion_model_ideal(x_noise_i, u, dt)
    x_noise_i = velocity_motion_model_noisy(x_ideal_i, u, dt, noise)
    ideal_model.append(x_ideal_i)
    noisy_model.append(x_noise_i)

ideal_model = np.array(ideal_model)
noisy_model = np.array(noisy_model)

fig, ax = plt.subplots()
ax.scatter(ideal_model[:, 0], ideal_model[:, 1], label="ideal model")
ax.plot(ideal_model[:, 0], ideal_model[:, 1])

ax.scatter(noisy_model[:, 0], noisy_model[:, 1], label="noisy model")
ax.plot(noisy_model[:, 0], noisy_model[:, 1])

plot_data(ideal_model, ax, 'g')
plot_data(noisy_model, ax, 'b')

plt.legend()
plt.show()
