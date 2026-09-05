import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime
from plot_util import init_plot
import logging
logging.basicConfig(
    level=logging.INFO,
    format="[%(levelname)s] %(message)s"
)

def main():

    args, data, ref_data, t, nplots, labels, save_path = init_plot(spatial=False)

    plt.figure(figsize=(10, 3 * nplots))

    for i in range(nplots):
        plt.subplot(nplots, 1, i + 1)
        plt.plot(t, data[:, i + 1], linewidth=2, label="data")
        plt.xlabel("t [s]")
        plt.ylabel(labels[i] if i < len(labels) else f"data {i}")
        plt.grid()
        
        if ref_data is not None:
            rmse = np.sqrt(np.mean((data[:, i + 1] - ref_data[:, i + 1]) ** 2))
            plt.plot(t, ref_data[:, i + 1], linewidth=2, label=f"reference, RMSE={rmse:.3f}", linestyle="--")
            plt.legend()

    plt.suptitle(args.title)
    plt.tight_layout()
    plt.savefig(save_path)
    plt.close()

    logging.info(f"File saved successfully to {save_path}")

if __name__ == "__main__":
    main()
