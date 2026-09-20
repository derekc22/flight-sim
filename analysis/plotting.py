import matplotlib.pyplot as plt


def save_figure(figure, save_path):
    figure.savefig(save_path, bbox_inches="tight")
    plt.close(figure)
    print(f"File saved successfully to {save_path}")
