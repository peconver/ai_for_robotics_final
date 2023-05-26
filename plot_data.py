import numpy as np
import matplotlib.pyplot as plt
from matplotlib import transforms

import os

my_dpi = 140
plt.figure(figsize=(500/my_dpi, 500/my_dpi), dpi=my_dpi)

def print_beacon_data(path):
    x = np.loadtxt(path, usecols=1, dtype=np.float32, delimiter=',')
    y = np.loadtxt(path, usecols=2, dtype=np.float32, delimiter=',')

    base = plt.gca().transData
    rot = transforms.Affine2D().rotate_deg(45)

    plt.plot(x, y, 'b-', transform = rot + base)
    plt.axis([-1, 1, 0, 2])
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    # plt.title('Robot going straight. Data measured with beacon, tag1. \n After rotation by 45 degrees to achieve approximately x=0 and y>=0')
    # plt.show()

def print_odometry_data(path):
    x = np.loadtxt(path, usecols=0, dtype=np.float32, delimiter=',')
    y = np.loadtxt(path, usecols=1, dtype=np.float32, delimiter=',')
    # yaw = np.loadtxt(path, usecols=range(3,4), dtype=np.float32, delimiter=',')



    base = plt.gca().transData
    rot = transforms.Affine2D().rotate_deg(70)


    

    plt.plot(x, y, 'r-', transform = rot + base)
    plt.axis([-1.5, 1, -1.5, 1])
    plt.axis('off')
    plt.xlabel('x [m]')
    plt.ylabel('y [m]')
    # plt.show()
    # plt.title('Robot going straight. Data measured with beacon, tag1. \n After rotation by 70 degrees to achieve approximately x=0 and y>=0')
   
    


if __name__ == "__main__":
    print_beacon = True
    print_odometry = True

    if (print_beacon):
        path_beacon = "../test/tag1/data.csv"
        print_beacon_data(path_beacon)

    if (print_odometry):
        path_odometry = "../odometry/data01.csv"
        print_odometry_data(path_odometry)

    plt.savefig("500.svg")
    plt.show()