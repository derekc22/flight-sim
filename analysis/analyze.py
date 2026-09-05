import matplotlib

matplotlib.use("Agg")

from analysis.config import load_json
from analysis.init import init_analysis
from analysis.linear.context import load_context
from analysis.linear.registry import RUNNERS
from analysis.validation import validate_analyses


def run_linear(config_dir_path, data_dir_path, plot_dir_path, report_dir_path, analyses):
    enabled = [(name, path) for name, path in analyses.items() if path is not None]
    if not enabled:
        return

    context = load_context(
        data_dir_path, plot_dir_path / "linear", report_dir_path / "linear"
    )
    print(f"Model loaded successfully from {data_dir_path / 'variables.json'}")

    for name, relative_config_path in enabled:
        RUNNERS[name](context, config_dir_path / relative_config_path)


def main():
    args = init_analysis()

    config = load_json(args.config_path, ("linear", "nonlinear"))

    validate_analyses(config)
    run_linear(
        args.config_path.parent,
        args.data_dir_path,
        args.plot_dir_path,
        args.report_dir_path,
        config["linear"],
    )


if __name__ == "__main__":
    main()
