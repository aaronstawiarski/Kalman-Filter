import numpy as np
import matplotlib.pyplot as plt

from ExampleKalmanFilter import KF

plt.ion()
plt.figure()

kf = KF(initial_x= 0.0, initial_v = 1.0 , accel_variance = 0.1)

DT = 0.1 # seconds
NUM_STEPS =  1000 # uppercase since this will stay constant
MEAS_N_STEPS = 20

real_x = 0.0
real_v = 0.5
meas_variance= 0.1 ** 2
mus = []
covs= []
real_xs = []
real_vs = []


for i in range (NUM_STEPS) :
    covs.append(kf.cov)
    mus.append(kf.mean)
    kf.predict(dt = DT)
    real_x = real_x + DT*real_v
    if i != 0 and i % MEAS_N_STEPS == 0:
        kf.update(meas_value = real_x + np.random.randn() * np.sqrt(meas_variance), meas_variance = meas_variance)
    real_xs.append(real_x)
    real_vs.append(real_v)


plt.figure()
plt.title('Position versus Time')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.plot([mu[0] for mu in mus], 'r', label = 'Estimated Position')
plt.plot(real_xs,'g', label = 'True Position')
plt.plot([mu[0] - 2*np.sqrt(cov[0,0]) for mu, cov in zip(mus,covs)], 'r--', label='-2σ')
plt.plot([mu[0] + 2*np.sqrt(cov[0,0]) for mu, cov in zip(mus,covs)], 'r--', label='+2σ')
plt.legend(loc = 'lower right')

plt.figure()
plt.title('Velocity versus Time')
plt.xlabel('Time (s)')
plt.ylabel('Position (m)')
plt.plot([mu[1] for mu in mus], 'b', label ='Estimated Velocity')
plt.plot(real_vs,'g',label = 'True Velocity')
plt.plot([mu[1] + 2*np.sqrt(cov[1,1]) for mu, cov in zip(mus,covs)], 'b--',label='-2σ')
plt.plot([mu[1] - 2*np.sqrt(cov[1,1]) for mu, cov in zip(mus,covs)], 'b--',label='+2σ')
plt.legend(loc = 'upper right')
plt.show()
plt.ginput(2)

print(covs)