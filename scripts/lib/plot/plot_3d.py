import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from plot_util import init_plot

def main():

    args, data, ref_data, _, _, labels, save_path = init_plot(spatial=True)

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection="3d")

    ax.plot(data[:, 1], data[:, 2], data[:, 3], linewidth=2, label="data")
    ax.scatter(data[0, 1], data[0, 2], data[0, 3], color="C1", s=50, label="start")
    ax.scatter(data[-1, 1], data[-1, 2], data[-1, 3], color="C2", s=50, label="end")

    ax.set_xlabel(labels[0] if 0 < len(labels) else "data 0")
    ax.set_ylabel(labels[1] if 1 < len(labels) else "data 1")
    ax.set_zlabel(labels[2] if 2 < len(labels) else "data 2")
    ax.grid()

    if ref_data is not None:
        ax.plot(ref_data[:, 1], ref_data[:, 2], ref_data[:, 3], linewidth=2, label="reference", linestyle="--")
        ax.scatter(ref_data[0, 1], ref_data[0, 2], ref_data[0, 3], color="C3", s=50, label="reference start")
        ax.scatter(ref_data[-1, 1], ref_data[-1, 2], ref_data[-1, 3], color="C4", s=50, label="reference end")

    ax.legend()

    plt.title(args.title)
    plt.tight_layout()
    plt.savefig(save_path)
    plt.close()

    print(f"File saved successfully to {save_path}")

if __name__ == "__main__":
    main()
