import control as ct
import matplotlib.pyplot as plt
import numpy as np

from analysis.config import load_json
from analysis.plotting import save_figure


CONFIG_FIELDS = (
    "time_sec",
    "step",
    "impulse",
    "ramp",
    "sine",
)


def first_in_band(error, tolerance, time):
    outside = np.flatnonzero(error > tolerance)
    return time[0] if outside.size == 0 else time[outside[-1] + 1]


def response_info(output, time):
    final_value = output[-1]
    error = np.abs(output - final_value)
    peak_index = np.argmax(np.abs(output))
    min_index = np.argmin(output)
    max_index = np.argmax(output)

    return {
        "transient_time": first_in_band(error, 0.02 * np.max(error), time),
        "settling_time": first_in_band(error, 0.02 * np.abs(final_value), time),
        "peak": np.abs(output[peak_index]),
        "peak_time": time[peak_index],
        "min": output[min_index],
        "min_time": time[min_index],
        "max": output[max_index],
        "max_time": time[max_index],
    }


def format_value(value):
    if np.isnan(value):
        return "NaN"
    if np.isposinf(value):
        return "Inf"
    if np.isneginf(value):
        return "-Inf"
    return f"{value:.10g}"


def write_response_info(file, outputs, time, file_prefix, input_index):
    response_title = file_prefix.replace("_", " ")
    file.write(f"{response_title}, input {input_index + 1}\n")
    file.write(f"{'-' * 60}\n")

    for output_index, output in enumerate(outputs):
        info = response_info(output, time)
        file.write(f"output {output_index + 1}\n")
        for field, value in info.items():
            file.write(f"  {field}: {format_value(value)}\n")
        file.write("\n")

    file.write("\n")


def make_input_signal(file_prefix, time, dt):
    if file_prefix == "step_response":
        return np.ones(time.size)
    if file_prefix == "impulse_response":
        signal = np.zeros(time.size)
        signal[0] = 1.0 / dt
        return signal
    if file_prefix == "ramp_response":
        return time
    if file_prefix == "sine_response":
        return np.sin(time)


def simulate_response(sys, time, inputs):
    return np.asarray(
        ct.forced_response(sys, timepts=time, inputs=inputs, squeeze=False).outputs
    )


def save_response_plots(outputs, time, plot_dir_path, file_prefix, input_index):
    response_title = file_prefix.replace("_", " ")

    for output_index, output in enumerate(outputs):
        figure, axes = plt.subplots()
        axes.plot(time, output, linewidth=2)
        axes.grid(True)
        axes.set_title(
            f"{response_title}: input {input_index + 1}, output {output_index + 1}"
        )
        axes.set_xlabel("time [s]")
        axes.set_ylabel(f"y_{output_index + 1}")

        filename = (
            f"{file_prefix}_input_{input_index + 1}_output_{output_index + 1}.pdf"
        )
        save_path = plot_dir_path / filename
        save_figure(figure, save_path)


def run(context, config_path):
    config = load_json(config_path, CONFIG_FIELDS)
    sys = context.sys
    dt = context.dt
    step_count = int(np.floor(config["time_sec"] / dt))
    time = np.arange(step_count + 1) * dt
    impulse_sys = ct.sample_system(sys, dt, method="zoh") if config["impulse"] else sys
    responses = (
        ("step_response", config["step"], sys),
        ("impulse_response", config["impulse"], impulse_sys),
        ("ramp_response", config["ramp"], sys),
        ("sine_response", config["sine"], sys),
    )

    info_path = context.report_dir_path / "time_domain_info.txt"
    with info_path.open("w", encoding="utf-8") as info_file:
        for file_prefix, enabled, response_sys in responses:
            if not enabled:
                continue

            signal = make_input_signal(file_prefix, time, dt)
            for input_index in range(sys.ninputs):
                inputs = np.zeros((sys.ninputs, time.size))
                inputs[input_index] = signal
                outputs = simulate_response(response_sys, time, inputs)
                save_response_plots(
                    outputs, time, context.plot_dir_path, file_prefix, input_index
                )
                write_response_info(
                    info_file, outputs, time, file_prefix, input_index
                )

    print(f"File saved successfully to {info_path}")
