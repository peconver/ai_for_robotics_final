import matplotlib.pyplot as plt
import numpy as np
import math
import matplotlib.pyplot as plt

# create a circle ride of diff drive
l = 0.160
steps = 200

# note: we are doing classification of systematic error for diagnosis, we need the r in paper
# note: error in robot versus error in odometry

class DifferentialDrive:
    def __init__(self, x0, y0, theta0, l, r_l, r_r):
        self.x = x0
        self.y = y0
        self.theta = theta0
        self.l = l
        self.r_r = r_r
        self.r_l = r_l
        self.T = 0.1

    def update(self, wR, wL):
        vR = wR * self.r_r
        vL = wL * self.r_l
        v_l = (vR + vL) / 2
        v_w = 2 * (vR - vL) / self.l
        dx = math.sin(self.theta) * v_l
        dy = math.cos(self.theta) * v_l

        dtheta = v_w
        self.x += dx
        self.y += dy
        self.theta += dtheta

        return self.x, self.y, self.theta

def plot_xy(row):
    x = []
    y = []
    for i in range(len(row)-1):
        if i % 2 == 0:
            x.append(row[i])
            y.append(row[i+1])
    plt.plot(x, y)

def plot_dataset(dataset1, dataset2 = []):
    # Plot each vector from the dataset
    x = []
    y = []
    for row in dataset1:
        plot_xy(row)
    if (dataset2):
        for row in dataset2:
            plot_xy(row)

    # Set labels, title, and legend
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Vectors x and y')
    plt.grid(True)
    plt.axis([-1, 1, 0, 2])
        
    plt.show()
    plt.legend()

    # Display the plot
    plt.show()

def plot_path(dataset1, dataset2, idx = 0):
    
    row = dataset1[idx]
    plot_xy(row)

    row = dataset2[idx]
    plot_xy(row)


    # Set labels, title, and legend
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Vectors x and y')
    plt.grid(True)
    plt.axis([-1, 1, 0, 2])
        
    plt.show()
    plt.legend()

    # Display the plot
    plt.show()

def print_2d_list_size(lst):
    num_rows = len(lst)
    num_columns = len(lst[0]) if lst else 0
    print("Size of the 2D list: {} rows x {} columns".format(num_rows, num_columns))

def generate_data(l, steps, s_err, label):
    dataset_m = []
    dataset_c = []
    p_m = np.zeros(steps*2)
    p_c = np.zeros(steps*2)

    labels = np.ones(len(s_err)) * label  # Exclude s_err = 0

    for e in s_err:
            odometry_calculation = DifferentialDrive(x0=0, y0=0, theta0=0, l=l, r_l=0.033 + e, r_r=0.033)
            beacon_measurement = DifferentialDrive(x0=0, y0=0, theta0=0, l=l, r_l=0.033, r_r=0.033)

            for i in range(steps):
                (x_m, y_m, theta) = beacon_measurement.update(wR=0.3, wL=0.3)
                p_m[i*2] = x_m + np.random.normal(0, 0.01)
                p_m[i*2 + 1] = y_m + np.random.normal(0, 0.01)

                (x_c, y_c, theta) = odometry_calculation.update(wR=0.3, wL=0.3)
                p_c[i*2] = x_c + np.random.normal(0, 0.01)
                p_c[i*2 + 1] = y_c + np.random.normal(0, 0.01)

            dataset_m.append(np.copy(p_m))
            dataset_c.append(np.copy(p_c))

    return dataset_m, dataset_c, labels

def combine_datasets(dataset_m, dataset_c, labels):
    X = np.concatenate((dataset_m, dataset_c), axis=1)
    dataset = np.column_stack((X, labels))
    return dataset

# generate data with error:
s_err = list( x/10000 for x in range(-10, 0, 1)) + list( x/10000 for x in range(1, 11, 1))
(de_m, de_c, le) = generate_data(l=l, steps=steps, s_err = s_err, label=1)
dataset_with_error = combine_datasets(dataset_m = de_m, dataset_c=de_c, labels=le)

# generate data without error:
(d_m, d_c, l) = generate_data(l = l, steps = steps, s_err = np.zeros(10), label=0)
dataset_without_error = combine_datasets(dataset_m = d_m, dataset_c=d_c, labels=l)


plot_dataset_with_error = False

if plot_dataset_with_error == True:
    plot_path(dataset1 = de_m, dataset2 = de_c, idx = np.random.randint(1, len(de_m)))
    plot_dataset(dataset1=de_c)


# export data as one dataset
dataset = np.concatenate((dataset_with_error, dataset_without_error), axis = 0)
# Print the size (shape) of dataset
print("Size of dataset:", dataset.shape)
np.savetxt('dataset.csv', dataset, delimiter=',')