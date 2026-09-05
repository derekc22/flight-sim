import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

from analysis.config import load_json
from analysis.linear.context import load_context
from analysis.linear.registry import RUNNERS
from analysis.validation import validate_analyses


def run_linear(project_path, output_dir, analyses):
    enabled = [(name, path) for name, path in analyses.items() if path is not None]
    if not enabled:
        return

    data_dir_path = project_path / "results" / output_dir / "data"
    plot_dir_path = project_path / "results" / output_dir / "figures"
    report_dir_path = project_path / "results" / output_dir / "reports"

    context = load_context(data_dir_path, plot_dir_path, report_dir_path)
    print(f"Model loaded successfully from {data_dir_path / 'variables.json'}")

    for name, relative_config_path in enabled:
        RUNNERS[name](context, project_path / "config" / relative_config_path)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("project_path", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()

    config = load_json(
        args.project_path / "config" / "analyze.json", ("linear", "nonlinear")
    )

    validate_analyses(config)
    run_linear(args.project_path, args.output_dir, config["linear"])


if __name__ == "__main__":
    main()
