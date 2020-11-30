
import matplotlib.pyplot as plt
import numpy as np
import os

import sys

def plot(file_path):

    file_name = os.path.basename(file_path).split('.csv')[0]

    fig_name = file_name + '.png'

    data = np.loadtxt(file_path, delimiter=',')

    t = data.T[0]
    t = t - t[0]
    latency = data.T[1]

    plt.plot(t, latency * 1.0e-6)
    plt.xlabel('Time [s]')
    plt.ylabel('Latency [us]')
    plt.savefig(fig_name)

if __name__ == '__main__':
    plot(sys.argv[1])
