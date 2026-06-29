#include <stdexcept>
#include <algorithm>
#include <cmath>
#include <format>
#include <nlohmann/json.hpp>
#include "core/json/public.hpp"
#include "core/json/runner/private.hpp"
#include "simulation/runner/public.hpp"
#include "simulation/constants/public.hpp"

namespace json {

    void validate_simulation_hz_json(const nlohmann::json& hz_json, const nlohmann::json& enable_json) {
		if (!hz_json.is_object()) { 
			throw std::runtime_error("json::validate_simulation_hz_json hz must be an object"); 
		}

		if (enable_json.at("avionics").get<bool>()) {
			if (!hz_json.contains("sensors")) {
	            throw std::runtime_error("json::validate_simulation_hz_json avionics requires sensors hz to be specified"); 
			}
			if (hz_json.at("sensors").get<double>() > constants::hz) {
	            throw std::runtime_error(std::format("json::validate_simulation_hz_json sensors hz cannot exceed simulation hz: {} hz", constants::hz)); 
			}
			if (!hz_json.contains("avionics")) {
	            throw std::runtime_error("json::validate_simulation_hz_json avionics requires avionics hz to be specified"); 
			}
			if (hz_json.at("avionics").get<double>() > constants::hz) {
	            throw std::runtime_error(std::format("json::validate_simulation_hz_json avionics hz cannot exceed simulation hz: {} hz", constants::hz)); 
			}
		}

		if (enable_json.at("estimation").get<bool>()) {
			if (!hz_json.contains("estimation")) {
	            throw std::runtime_error("json::validate_simulation_hz_json estimation requires estimation hz to be specified"); 
			}
			if (hz_json.at("estimation").get<double>() > constants::hz) {
	            throw std::runtime_error(std::format("json::validate_simulation_hz_json estimation hz cannot exceed simulation hz: {} hz", constants::hz)); 
			}
		}

		if (enable_json.at("control").get<bool>()) {
			if (!hz_json.contains("guidance")) {
	            throw std::runtime_error("json::validate_simulation_hz_json control requires guidance hz to be specified"); 
			}
			if (hz_json.at("guidance").get<double>() > constants::hz) {
	            throw std::runtime_error(std::format("json::validate_simulation_hz_json guidance hz cannot exceed simulation hz: {} hz", constants::hz)); 
			}
			if (!hz_json.contains("control")) {
	            throw std::runtime_error("json::validate_simulation_hz_json control requires control hz to be specified"); 
			}
			if (hz_json.at("control").get<double>() > constants::hz) {
	            throw std::runtime_error(std::format("json::validate_simulation_hz_json control hz cannot exceed simulation hz: {} hz", constants::hz)); 
			}
		}
	}

    void validate_enable_json(const nlohmann::json& enable_json) {
		if (!enable_json.is_object()) { 
			throw std::runtime_error("json::validate_enable_json enable must be an object"); 
		}
	}

    void validate_simulation_json(const nlohmann::json& simulation_json) {
		double time_sec = simulation_json.at("time_sec").get<double>();
		if (!std::isfinite(time_sec) || time_sec <= 0.0) {
			throw std::runtime_error("json::parse_runner_config time_sec must be finite and non-negative");
		}

        const auto& enable_json = simulation_json.at("enable");
		validate_enable_json(enable_json);

        const auto& hz_json = simulation_json.at("hz");
		validate_simulation_hz_json(hz_json, enable_json);
	}

	void validate_logging_hz(double hz, const nlohmann::json& enable_json) {
		if (enable_json.at("verbose").get<bool>()) {
			if (!hz) {
				throw std::runtime_error("json::validate_logging_hz verbose requires logging hz to be specified"); 
			}
		}
		if (enable_json.at("rerun").get<bool>()) {
			if (!hz) {
				throw std::runtime_error("json::validate_logging_hz rerun requires logging hz to be specified"); 
			}
		}
		if (hz > constants::hz) {
			throw std::runtime_error(std::format("json::validate_logging_hz hz cannot exceed simulation hz: {} hz", constants::hz)); 
		}
	}

    void validate_logging_json(const nlohmann::json& logging_json) {
        double hz = logging_json.at("hz").get<double>();

        const auto& enable_json = logging_json.at("enable");
		validate_enable_json(enable_json);

		validate_logging_hz(hz, enable_json);
	}

    runner::JSONOptions parse_runner_options(const nlohmann::json& config) {
        const auto& simulation_json = config.at("simulation");
		validate_simulation_json(simulation_json);

        const auto& logging_json = config.at("logging");
		validate_logging_json(logging_json);

		// compute number of simulation steps
		double time_sec = simulation_json.at("time_sec").get<double>();
		int tf = std::max(1, static_cast<int>(std::ceil(time_sec / constants::dt)));

		const auto& simulation_hz_json = simulation_json.at("hz");
		const auto& simulation_enable_json = simulation_json.at("enable");

		double logging_hz = logging_json.at("hz");
		const auto& logging_enable_json = logging_json.at("enable");

		return {
			.tf=tf,

			.sensor_hz = simulation_hz_json.at("sensors").get<double>(),
			.avionics_hz = simulation_hz_json.at("avionics").get<double>(),
			.estimation_hz = simulation_hz_json.at("estimation").get<double>(),
			.guidance_hz = simulation_hz_json.at("guidance").get<double>(),
			.control_hz = simulation_hz_json.at("control").get<double>(),

			.trim_bool=simulation_enable_json.at("trim").get<bool>(),
			.avionics_bool=simulation_enable_json.at("avionics").get<bool>(),
			.control_bool=simulation_enable_json.at("control").get<bool>(),
			.estimation_bool=simulation_enable_json.at("estimation").get<bool>(),
			.wind_bool=simulation_enable_json.at("wind").get<bool>(),

			.logging_hz = logging_hz,

			.verbose_bool=logging_enable_json.at("verbose").get<bool>(),
			.rerun_bool=logging_enable_json.at("rerun").get<bool>()
		};

    }

}
