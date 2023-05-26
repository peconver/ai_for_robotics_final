import matplotlib.pyplot as plt
import numpy as np
import math

def diff_drive(x0, y0, theta0, vR, vL, l):
  v_l = (vR + vL)/2
  v_w = 2*(vR - vL)/l
  dx = math.cos(theta) * v_l
  dy = math.sin(theta) * v_w
  dtheta = v_w
  return dx, dy, dtheta

# EXPERIMENT
x = 0
y = 0
theta = 0.8
dt = 0.1
l = 1

x_gt = 0
y_gt = 0
theta_gt = 0
dt = 0.1
l_gt = 1

steps = 1000
features_per_step = 4
dataset = np.zeros([steps, features_per_step])
# print(dataset)

dataset_gt = np.zeros([steps, features_per_step])
print(dataset_gt)
for i in range(steps):
  # Expected kinemactic model, it is expected that: R_L = R_R
  vR = i/100
  vL = i/100
  dx, dy, dtheta = diff_drive(x, y, theta, vR, vL, l)

  # "Real" kinematic model, R_L != R_R
  vR_gt = (i-1)/100
  vL_gt = i/100
  dx_gt, dy_gt, dtheta_gt = diff_drive(x_gt, y_gt, theta_gt, vR_gt, vL_gt, l_gt)

  # Integration approximated with summation
  # for expected kinematic model and "Real" kinematic model
  x = x + dx*dt
  y = y + dy*dt
  theta = theta + dtheta*dt

  x_gt = x_gt + dx_gt*dt
  y_gt = y_gt + dy_gt*dt
  theta_gt = theta_gt + dtheta_gt*dt

  # storing values to dataset
  dataset[i, 0] = x
  dataset[i, 1] = y
  dataset[i, 2] = theta
#   print(x)

  # ADD NOISE later - in case of too high accuracy of the NN
  dataset[i, 0] = x + np.random.normal(0, 0.01) # generates a random number from a normal distribution with a mean of 0 and a standard deviation of 0.1.
  dataset[i, 1] = y + np.random.normal(0, 0.01)
  dataset[i, 2] = theta + np.random.normal(0, 0.1)
  dataset_gt[i, 0] = x_gt+np.random.normal(0, 0.01)
  dataset_gt[i, 1] = y_gt+np.random.normal(0, 0.01)
  dataset_gt[i, 2] = theta_gt+np.random.normal(0, 0.01)


#reshape dataset to the form: x1, y1, theta1, ..., xsteps, ysteps, thetasteps, x_gt1, y_gt1, theta_gt1,...,x_gtsteps, y_gtsteps, theta_gtsteps, label
dataset = np.reshape(dataset, [steps, features_per_step])
dataset_gt = np.reshape(dataset_gt, [steps, features_per_step])

#add label (0 - no error, 1 - errpr)
dataset = np.append(dataset, np.zeros([steps, 1]), axis=1)
dataset_gt = np.append(dataset_gt, np.ones([steps, 1]), axis=1)
#merge datasets
dataset = np.append(dataset, dataset_gt, axis=0)
#shuffle dataset
np.random.shuffle(dataset)
#export to csv
np.savetxt("dataset.csv", dataset, delimiter=",")
np.savetxt("dataset_gt.csv", dataset_gt, delimiter=",")



# plot the path
plt.plot(dataset[:, 0], dataset[:, 1])
plt.plot(dataset_gt[:, 0], dataset_gt[:, 1])
plt.show()

label = 10 # error in radius
'''TODO:
- edit method to produce lot of different experiments, which means experiments
 with different scale of errors and different noise
- 
- add label 
- export to csv 
- create machine learning framework.............
'''