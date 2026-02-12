import numpy as np
import matplotlib.pyplot as plt
import os
import argparse
from datetime import datetime

parser = argparse.ArgumentParser
parser.add_argument("")
parser.add_argument("")


data_dir = "data/hw3"
save_dir = "figures/hw3"
save_ext = "pdf"

os.makedirs(save_dir, exist_ok=True)
dtn = datetime.now().strftime("%Y%b%d_%H-%M-%S")

for fname in os.listdir(data_dir):
    data_path = os.path.join(data_dir, fname)
    save_fname = os.path.splitext(fname)[0]
    # save_fpath = os.path.join(save_dir, f"{save_fname}_{dtn}.{save_ext}")
    save_fpath = os.path.join(save_dir, f"{save_fname}.{save_ext}")

    data = np.loadtxt(data_path, delimiter=",")
    t = np.linspace(0, data[-1, 0], data.shape[0])

    plt.subplot(3,1,1)
    plt.plot(t, data[:, 1], linewidth=2)
    plt.xlabel('t [s]')
    plt.ylabel('x(t) [m]')
    plt.grid()

    plt.subplot(3,1,2)
    plt.plot(t,  data[:, 2], linewidth=2)
    plt.xlabel('t [s]')
    plt.ylabel('y(t) [m]')
    plt.grid()

    plt.subplot(3,1,3)
    plt.plot(t, data[:, 3], linewidth=2)
    plt.xlabel('t [s]')
    plt.ylabel('z(t) [m]')    
    plt.grid()

    plt.suptitle("x(t), y(t), z(t)")
    plt.tight_layout()
    plt.savefig(save_fpath)
    plt.close()
