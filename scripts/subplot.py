import numpy as np
import matplotlib.pyplot as plt
import os
import argparse
from datetime import datetime

parser = argparse.ArgumentParser(description="Plotting CLI arguments")
parser.add_argument("--data", "-d", type=str, help="Data file path")
parser.add_argument("--ref", "-r", type=str, help="Reference data file path")
parser.add_argument("--ext", "-e", type=str, help="Plot extension", default="pdf")
parser.add_argument("--labels", "-l", nargs="+", type=str ,help="List of plot labels")
parser.add_argument("--title", "-t", type=str, help="Title")
args = parser.parse_args() 


data_fpath = f"data/{args.data}"
ref_data_fpath = f"data/{args.ref}" if args.ref is not None else None
save_dir = f"figures/{os.path.basename(os.path.dirname(data_fpath))}"
os.makedirs(save_dir, exist_ok=True)
dtn = datetime.now().strftime("%Y%b%d_%H-%M-%S")

save_fname = os.path.splitext(os.path.basename(data_fpath))[0]
save_fpath = os.path.join(save_dir, f"{save_fname}.{args.ext}")

data = np.loadtxt(data_fpath, delimiter=",")
ref_data = np.loadtxt(ref_data_fpath, delimiter=",") if ref_data_fpath is not None else None
t = np.linspace(0, data[-1, 0], data.shape[0])

nplots = data.shape[1] - 1
labels = args.labels[:nplots]

plt.figure(figsize=(10, 3 * nplots))

for i in range(nplots):
    plt.subplot(nplots, 1, i + 1)
    plt.plot(t, data[:, i + 1], linewidth=2, label="data")
    plt.xlabel("t [s]")
    plt.ylabel(labels[i] if i < len(labels) else f"data {i}")
    plt.grid()
    
    if ref_data is not None:
        plt.plot(t, ref_data[:, i + 1], linewidth=2, label="setpoint", linestyle="--")
        plt.legend()

plt.suptitle(args.title)
plt.tight_layout()
plt.savefig(save_fpath)
plt.close()

print(f"File saved successfully to {save_fpath}")