#visualize dataset x, y, yaw

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import math
import csv

#read data from csv file in two columns
x = []
y = []
yaw = []
data = pd.read_csv('data.csv', delimiter='\t')
x = data[0]
y = data[1]
yaw = data[2]


