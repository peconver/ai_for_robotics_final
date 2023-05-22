import numpy as np
import matplotlib.pyplot as plt
from matplotlib import transforms

import os

# read data from file
print("PATH", os.getcwd())
path = "test/tag1/data.csv"
x = np.loadtxt(path, usecols=range(1, 2), dtype=np.float32, delimiter=',')
y = np.loadtxt(path, usecols=range(2, 3), dtype=np.float32, delimiter=',')

base = plt.gca().transData
rot = transforms.Affine2D().rotate_deg(45)

# define transformed line
# line = plt.plot(data, 'r--', transform= rot + base)
# print(x)
plt.plot(x, y, 'rx', transform = rot + base)
plt.axis([-1, 1, 0, 2])
plt.xlabel('x [m]')
plt.ylabel('y [m]')
plt.title('Robot going straight. Data measured with beacon, tag1. \n After rotation by 45 degrees to achieve approximately x=0 and y>=0')
plt.show()

# :)
