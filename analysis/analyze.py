import argparse
from pathlib import Path

from analysis.config import load_json
from analysis.linear.model import load_model
from analysis.linear.registry import RUNNERS, LinearAnalysisContext


def run_linear(project_path, output_dir, analyses):
    enabled = [(name, path) for name, path in analyses.items() if path is not None]
    if not enabled:
        return

    data_dir_path = project_path / "results" / output_dir / "data"
    plot_dir_path = project_path / "results" / output_dir / "figures"
    report_dir_path = project_path / "results" / output_dir / "reports"

    sys, dt = load_model(data_dir_path)
    print(f"Model loaded successfully from {data_dir_path / 'variables.json'}")

    context = LinearAnalysisContext(
        sys, dt, plot_dir_path, report_dir_path
    )

    for name, relative_config_path in enabled:
        if name not in RUNNERS:
            raise ValueError(f"No linear analysis implementation for '{name}'")

        RUNNERS[name](context, project_path / "config" / relative_config_path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("project_path", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()

    config = load_json(
        args.project_path / "config" / "analyze.json", ("linear", "nonlinear")
    )

    run_linear(args.project_path, args.output_dir, config.get("linear", {}))

    nonlinear = config.get("nonlinear", {})
    if any(path is not None for path in nonlinear.values()):
        raise ValueError("No nonlinear analysis implementations are available")


if __name__ == "__main__":
    main()
