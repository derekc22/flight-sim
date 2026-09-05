from pathlib import Path

import control as ct
import numpy as np

from analysis.config import load_json


VARIABLE_FIELDS = (
    "aircraft_id",
    "hz.hz",
    "hz.control_hz",
    "state_dim",
    "input_dim",
    "matrix_paths.A",
    "matrix_paths.B",
    "matrix_paths.C",
    "matrix_paths.D",
)


def load_model(data_dir_path):
    data_dir_path = Path(data_dir_path)
    variables = load_json(data_dir_path / "variables.json", VARIABLE_FIELDS)
    matrix_paths = variables["matrix_paths"]

    matrices = [
        np.loadtxt(data_dir_path / matrix_paths[name], delimiter=",", ndmin=2)
        for name in ("A", "B", "C", "D")
    ]

    return ct.ss(*matrices), 1.0 / variables["hz"]["hz"]
