import matplotlib

matplotlib.use("Agg")

import control as ct
import numpy as np

from analysis.config import load_json
from analysis.plotting import save_figure


CONFIG_FIELDS = (
    "w_min_order",
    "w_max_order",
    "num_points",
    "bode",
    "nyquist",
    "nichols",
    "singular_value",
)


def plot_siso_responses(
    sys, omega, plot_dir_path, file_prefix, plot_response, cartesian_grid=False
):
    response_title = file_prefix.replace("_", " ")

    for input_index in range(sys.ninputs):
        for output_index in range(sys.noutputs):
            title = (
                f"{response_title}: input {input_index + 1}, output {output_index + 1}"
            )
            control_plot = plot_response(
                sys[output_index, input_index], omega, title
            )
            if cartesian_grid:
                for axes in control_plot.axes.flat:
                    axes.grid(True)
            filename = (
                f"{file_prefix}_input_{input_index + 1}_output_{output_index + 1}.pdf"
            )
            save_figure(control_plot.figure, plot_dir_path / filename)


def run(context, config_path):
    config = load_json(config_path, CONFIG_FIELDS)
    sys = context.sys
    omega = np.logspace(
        config["w_min_order"], config["w_max_order"], config["num_points"]
    )

    if config["bode"]:
        plot_siso_responses(
            sys,
            omega,
            context.plot_dir_path,
            "bode_response",
            lambda response, frequencies, title: ct.bode_plot(
                response,
                omega=frequencies,
                dB=True,
                deg=True,
                grid=True,
                title=title,
            ),
        )

    if config["nyquist"]:
        plot_siso_responses(
            sys,
            omega,
            context.plot_dir_path,
            "nyquist_response",
            lambda response, frequencies, title: ct.nyquist_plot(
                response, omega=frequencies, title=title
            ),
            cartesian_grid=True,
        )

    if config["nichols"]:
        plot_siso_responses(
            sys,
            omega,
            context.plot_dir_path,
            "nichols_response",
            lambda response, frequencies, title: ct.nichols_plot(
                response, omega=frequencies, grid=True, title=title
            ),
        )

    if config["singular_value"]:
        control_plot = ct.singular_values_plot(
            sys,
            omega=omega,
            dB=True,
            grid=True,
            title="singular value response",
        )
        save_figure(
            control_plot.figure,
            context.plot_dir_path / "singular_value_response.pdf",
        )
