from pathlib import Path

import control as ct
import numpy as np


def load_model(data_dir_path, matrix_paths):
    data_dir_path = Path(data_dir_path)

    matrices = [
        np.loadtxt(data_dir_path / matrix_paths[name], delimiter=",", ndmin=2)
        for name in ("A", "B", "C", "D")
    ]

    return ct.ss(*matrices)
