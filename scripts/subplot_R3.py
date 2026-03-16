import numpy as np
import matplotlib.pyplot as plt
import os
import argparse
from datetime import datetime

parser = argparse.ArgumentParser(description="Plotting CLI arguments")
parser.add_argument("--data", "-d", type=str, help="Directory name with data files")
parser.add_argument("--ext", "-e", type=str, help="Plot extension", default="pdf")
parser.add_argument("--label0", "-l0", type=str, help="Data 0 label", default="x(t) [m]")
parser.add_argument("--label1", "-l1", type=str, help="Data 1 label", default="y(t) [m]")
parser.add_argument("--label2", "-l2", type=str, help="Data 2 label", default="z(t) [m]")
parser.add_argument("--title", "-t", type=str, help="Title", default="x(t), y(t), z(t)")
args = parser.parse_args()


data_dir = f"data/{args.data}"
save_dir = f"figures/{args.data}"

os.makedirs(save_dir, exist_ok=True)
dtn = datetime.now().strftime("%Y%b%d_%H-%M-%S")

for fname in os.listdir(data_dir):
    data_path = os.path.join(data_dir, fname)
    save_fname = os.path.splitext(fname)[0]
    save_fpath = os.path.join(save_dir, f"{save_fname}.{args.ext}")

    data = np.loadtxt(data_path, delimiter=",")
    t = np.linspace(0, data[-1, 0], data.shape[0])

    plt.figure(figsize=(10, 9))

    plt.subplot(3,1,1)
    plt.plot(t, data[:, 1], linewidth=2)
    plt.xlabel("t [s]")
    plt.ylabel(args.label0)
    plt.grid()

    plt.subplot(3,1,2)
    plt.plot(t,  data[:, 2], linewidth=2)
    plt.xlabel("t [s]")
    plt.ylabel(args.label1)
    plt.grid()

    plt.subplot(3,1,3)
    plt.plot(t, data[:, 3], linewidth=2)
    plt.xlabel("t [s]")
    plt.ylabel(args.label2)    
    plt.grid()

    plt.suptitle(args.title)
    plt.tight_layout()
    plt.savefig(save_fpath)
    plt.close()
    print("Plots saved successfully")
