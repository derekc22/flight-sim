import numpy as np
import matplotlib.pyplot as plt
import os
import argparse
from datetime import datetime

parser = argparse.ArgumentParser(description="Plotting CLI arguments")
parser.add_argument("--data", "-d", type=str, help="Directory name with data files")
parser.add_argument("--ext", "-e", type=str, help="Plot extension", default="pdf")
parser.add_argument("--labels", "-l", nargs="+", type=str ,help="List of data labels")
parser.add_argument("--title", "-t", type=str, help="Title")
args = parser.parse_args() 
# usage: 
# python3 scripts/subplot.py -d hw5/position -l "x(t) [m]" "y(t) [m]" "z(t) [m]" -t "position vs time";
# python3 scripts/subplot.py -d hw5/aero_force -l "Fx(t) [N]" "Fy(t) [N]" "Fz(t) [N]" -t "aero force vs time";
# python3 scripts/subplot.py -d hw5/aero_moment -l "Mx(t) [Nm]" "My(t) [Nm]" "Mz(t) [Nm]" -t "aero moment vs time";
# python3 scripts/subplot.py -d hw5/aero_state -l "Vinf(t) [m/s]" "α(t) [rad]" "β(t) [rad]" -t "aero properties vs time";


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

    nplots = data.shape[1] - 1
    labels = args.labels[:nplots]

    plt.figure()

    for i in range(nplots):
        plt.subplot(nplots, 1, i + 1)
        plt.plot(t, data[:, i + 1], linewidth=2)
        plt.xlabel("t [s]")
        plt.ylabel(labels[i] if i < len(labels) else f"data {i}")
        plt.grid()

    plt.suptitle(args.title)
    plt.tight_layout()
    plt.savefig(save_fpath)
    plt.close()