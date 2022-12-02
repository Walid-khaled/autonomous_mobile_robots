import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation

print("Initial belief of the network")

bel_Xc = 0.5  # bel(X=free)
bel_X = np.array([bel_Xc, 1-bel_Xc])  # % bel(X=free), bel(X=obs)

print("Conditional probabilities of the image processing technique")
P_ZcXc = 0.8      # P(Z=free|X=free)
P_ZdXc = 1-P_ZcXc  # P(Z=obs|X=free)
P_ZdXd = 0.9      # P(Z=obs|X=obs)
P_ZcXd = 1-P_ZdXd  # P(Z=free|X=obs)
p_ZX = np.array([[P_ZcXc, P_ZcXd], [P_ZdXc, P_ZdXd]])

print('Outcome probabilities when applying control command')
P_XcXcUc = 1          # P(X=free|X'=free,U'=move)
P_XdXcUc = 1-P_XcXcUc  # P(X=obs|X'=free,U'=move)
P_XcXdUc = 0.8        # P(X=free|X'=obs,U'=move)
P_XdXdUc = 1-P_XcXdUc  # P(X=obs|X'=obs,U'=move)
p_ZXUc = np.array([[P_XcXcUc, P_XdXcUc], [P_XcXdUc, P_XdXdUc]])

print('Outcome probabilities when not applying control command')
P_XcXcUn = 1          # P(X=free|X'=free,U'=not_move)
P_XdXcUn = 1-P_XcXcUn  # P(X=obs|X'=free,U'=not_move)
P_XcXdUn = 0          # P(X=free|X'=obs,U'=not_move)
P_XdXdUn = 1-P_XcXdUn  # P(X=obs|X'=obs,U'=not_move)
p_ZXUn = np.array([[P_XcXcUn, P_XdXcUn], [P_XcXdUn, P_XdXdUn]])

U = np.array(['not_move',  'move', 'move'])
Z = np.array(['obs', 'free', 'free'])

bel_X_steps_prediction = []
bel_X_steps_correction = []
for idx, u, z in zip(range(0, len(U)), U, Z):
    print('Prediction step: U({0})={1}'.format(idx, u))
    if u == 'move':
        belp_X = bel_X@p_ZXUc
    else:
        belp_X = bel_X@p_ZXUn
    bel_X_steps_prediction.append(belp_X)
    print("belp_X: ", belp_X)
    print('Correction step: Z({0}))={1}'.format(idx, z))
    if z == 'free':
        bel_X = p_ZX[0]*belp_X
    else:
        bel_X = p_ZX[1]*belp_X
    bel_X = bel_X/np.sum(bel_X)
    bel_X_steps_correction.append(bel_X)
    print("bel_X: ", bel_X)

bel_X_steps_prediction = np.array(bel_X_steps_prediction)
bel_X_steps_correction = np.array(bel_X_steps_correction)

def barlist(n):
    return np.array([bel_X_steps_prediction[n], bel_X_steps_correction[n]]).flatten()

fig = plt.figure()
n = len(U)-1
x = range(0, bel_X.shape[0]*2)
print(x)

barcollection = plt.bar(x, barlist(0))

def animate(i):
    y = barlist(i+1)
    for i, b in enumerate(barcollection):
        print(i, b)
        b.set_height(y[i])

anim = animation.FuncAnimation(fig, animate, repeat=False, blit=False, frames=n,
                               interval=1000)
plt.ylabel("Probability")
plt.title('bel_X after prediction step and correction step')
# anim.save('bel_x.mp4', writer=animation.FFMpegWriter(fps=10))
plt.show()