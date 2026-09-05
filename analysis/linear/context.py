from dataclasses import dataclass
from pathlib import Path

import control as ct

from analysis.config import load_json
from analysis.linear.model import load_model


REQUIRED_VARIABLE_FIELDS = (
    "aircraft_id",
    "hz.hz",
    "hz.control_hz",
    "matrix_paths.A",
    "matrix_paths.B",
    "matrix_paths.C",
    "matrix_paths.D",
)


@dataclass
class LinearAnalysisContext:
    aircraft_id: str
    control_hz: float
    state_dim: int
    input_dim: int
    sys: ct.StateSpace
    dt: float
    plot_dir_path: Path
    report_dir_path: Path


def load_context(data_dir_path, plot_dir_path, report_dir_path):
    variables = load_json(
        data_dir_path / "variables.json", REQUIRED_VARIABLE_FIELDS
    )
    sys = load_model(data_dir_path, variables["matrix_paths"])

    return LinearAnalysisContext(
        aircraft_id=variables["aircraft_id"],
        control_hz=variables["hz"]["control_hz"],
        state_dim=sys.A.shape[0],
        input_dim=sys.B.shape[1],
        sys=sys,
        dt=1.0 / variables["hz"]["hz"],
        plot_dir_path=plot_dir_path,
        report_dir_path=report_dir_path,
    )
