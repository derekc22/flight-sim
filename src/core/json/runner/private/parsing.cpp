#include <algorithm>
#include <cmath>
#include <nlohmann/json.hpp>
#include "core/json/runner/private/parsing.hpp"
#include "core/json/runner/private/validation.hpp"
#include "simulation/runner/public/data/types.hpp"
#include "simulation/constants/public/scalars.hpp"

namespace json {

	runner::JSONOptions parse_runner_options(
	    const nlohmann::json& config)
	{
		const auto& simulation_json = config.at("simulation");
		validate_simulation_json(simulation_json);

		const auto& logging_json = config.at("logging");
		validate_logging_json(logging_json);

		// compute number of simulation steps
		double time_sec = simulation_json.at("time_sec").get<double>();
		int tf = std::max(1, static_cast<int>(std::ceil(time_sec / constants::dt)));

		const auto& simulation_hz_json = simulation_json.at("hz");
		const auto& simulation_enable_json = simulation_json.at("enable");

		const auto& logging_enable_json = logging_json.at("enable");

		runner::ModuleRates module_rates{
		    .sensor_hz = simulation_enable_json.at("avionics").get<bool>()
		        ? simulation_hz_json.at("sensors").get<double>()
		        : 0.0,
		    .avionics_hz = simulation_enable_json.at("avionics").get<bool>()
		        ? simulation_hz_json.at("avionics").get<double>()
		        : 0.0,
		    .estimation_hz = simulation_enable_json.at("estimation").get<bool>()
		        ? simulation_hz_json.at("estimation").get<double>()
		        : 0.0,
		    .guidance_hz = simulation_enable_json.at("control").get<bool>()
		        ? simulation_hz_json.at("guidance").get<double>()
		        : 0.0,
		    .control_hz =
		        simulation_enable_json.at("control").get<bool>() ? simulation_hz_json.at("control").get<double>() : 0.0,
		    .log_hz = (logging_enable_json.at("verbose").get<bool>() || logging_enable_json.at("rerun").get<bool>())
		        ? logging_json.at("hz").get<double>()
		        : 0.0,
		};

		runner::JSONFlags json_flags{.trim_flag = simulation_enable_json.at("trim").get<bool>(),
		    .avionics_flag = simulation_enable_json.at("avionics").get<bool>(),
		    .control_flag = simulation_enable_json.at("control").get<bool>(),
		    .estimation_flag = simulation_enable_json.at("estimation").get<bool>(),
		    .wind_flag = simulation_enable_json.at("wind").get<bool>(),
		    .verbose_flag = logging_enable_json.at("verbose").get<bool>(),
		    .rerun_flag = logging_enable_json.at("rerun").get<bool>(),
		    .joystick_flag = simulation_enable_json.at("joystick").get<bool>()};

		return {.tf = tf, .flags = json_flags, .module_rates = module_rates};
	}

} // namespace json
