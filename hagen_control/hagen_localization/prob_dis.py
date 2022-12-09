from scipy.stats import norm
import numpy as np
import matplotlib.pyplot as plt

x = np.linspace(-20, 20, 100)

mean = 10
std = 1
dis = norm(mean, std)

plt.plot(x, dis.pdf(x))
plt.show()