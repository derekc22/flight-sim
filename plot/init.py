import numpy as np
import os
import argparse
from datetime import datetime

def init_plot(spatial=False):
    parser = argparse.ArgumentParser(description="Plotting CLI arguments")
    parser.add_argument("--data", "-d", type=str, help="Data file path")
    parser.add_argument("--ref", "-r", type=str, help="Reference data file path")
    parser.add_argument("--save", "-s", type=str, help="Plot save directory")
    parser.add_argument("--ext", "-e", type=str, help="Plot extension", default="pdf")
    parser.add_argument("--labels", "-l", nargs="+", type=str ,help="List of plot labels")
    parser.add_argument("--title", "-t", type=str, help="Title")
    args = parser.parse_args() 

    data_path = args.data
    ref_data_path = args.ref
    plot_dir_path = args.save
    dtn = datetime.now().strftime("%Y%b%d_%H-%M-%S")

    save_fname = os.path.splitext(os.path.basename(data_path))[0]
    if spatial: 
        save_fname += "_3d" 
    save_path = os.path.join(plot_dir_path, f"{save_fname}.{args.ext}")

    data = np.loadtxt(data_path, delimiter=",")
    ref_data = np.loadtxt(ref_data_path, delimiter=",") if ref_data_path is not None else None
    t = np.linspace(0, data[-1, 0], data.shape[0])

    nplots = data.shape[1] - 1
    labels = args.labels[:nplots]

    return args, data, ref_data, t, nplots, labels, save_path
