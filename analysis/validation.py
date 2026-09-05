from analysis.linear.registry import RUNNERS


def validate_analyses(config):
    for name, path in config["linear"].items():
        if path is not None and name not in RUNNERS:
            raise ValueError(f"No linear analysis implementation for '{name}'")

    if any(path is not None for path in config["nonlinear"].values()):
        raise ValueError("No nonlinear analysis implementations are available")
