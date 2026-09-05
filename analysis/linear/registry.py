from dataclasses import dataclass
from pathlib import Path

import control as ct

from analysis.linear.frequency_domain import run as run_frequency_domain
from analysis.linear.time_domain import run as run_time_domain


@dataclass
class LinearAnalysisContext:
    sys: ct.StateSpace
    dt: float
    plot_dir_path: Path
    report_dir_path: Path


RUNNERS = {
    "time_domain": run_time_domain,
    "frequency_domain": run_frequency_domain,
}
