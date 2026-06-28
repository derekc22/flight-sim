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

    void validate_hz_json(const nlohmann::json& hz_config, const nlohmann::json& enable_config) {
        if (hz_config.contains("hz")) {
            const auto& hz_json = hz_config.at("hz");
            if (!hz_json.is_object()) { 
                throw std::runtime_error("json::validate_hz_json hz must be an object"); 
            }
		}

		if (enable_config.at("avionics").get<bool>()) {
			if (!hz_config.contains("sensors")) {
	            throw std::runtime_error("json::validate_hz_json avionics requires sensors hz to be specified"); 
			}
			if (hz_config.at("sensors").get<double>() > constants::hz) {
	            throw std::runtime_error(std::format("json::validate_hz_json sensors hz cannot exceed simulation hz: {} hz", constants::hz)); 
			}
			if (!hz_config.contains("avionics")) {
	            throw std::runtime_error("json::validate_hz_json avionics requires avionics hz to be specified"); 
			}
			if (hz_config.at("avionics").get<double>() > constants::hz) {
	            throw std::runtime_error(std::format("json::validate_hz_json avionics hz cannot exceed simulation hz: {} hz", constants::hz)); 
			}
		}

		if (enable_config.at("estimation").get<bool>()) {
			if (!hz_config.contains("estimation")) {
	            throw std::runtime_error("json::validate_hz_json estimation requires estimation hz to be specified"); 
			}
			if (hz_config.at("estimation").get<double>() > constants::hz) {
	            throw std::runtime_error(std::format("json::validate_hz_json estimation hz cannot exceed simulation hz: {} hz", constants::hz)); 
			}
		}

		if (enable_config.at("control").get<bool>()) {
			if (!hz_config.contains("guidance")) {
	            throw std::runtime_error("json::validate_hz_json control requires guidance hz to be specified"); 
			}
			if (hz_config.at("guidance").get<double>() > constants::hz) {
	            throw std::runtime_error(std::format("json::validate_hz_json guidance hz cannot exceed simulation hz: {} hz", constants::hz)); 
			}
			if (!hz_config.contains("control")) {
	            throw std::runtime_error("json::validate_hz_json control requires control hz to be specified"); 
			}
			if (hz_config.at("control").get<double>() > constants::hz) {
	            throw std::runtime_error(std::format("json::validate_hz_json control hz cannot exceed simulation hz: {} hz", constants::hz)); 
			}
		}
	}

    void validate_enable_json(const nlohmann::json& enable_config) {
        if (enable_config.contains("enable")) {
            const auto& enable_json = enable_config.at("enable");
            if (!enable_json.is_object()) { 
                throw std::runtime_error("json::validate_enable_json enable must be an object"); 
            }
		}
	}

    runner::JSONOptions parse_runner_options(const nlohmann::json& config) {
		double time_sec = config.at("time_sec").get<double>();
		if (!std::isfinite(time_sec) || time_sec <= 0.0) {
			throw std::runtime_error("json::parse_runner_config time_sec must be finite and non-negative");
		}
		// compute number of simulation steps
		int tf = std::max(1, static_cast<int>(std::ceil(time_sec / constants::dt)));

        const auto& enable_json = config.at("enable");
		validate_enable_json(enable_json);

        const auto& hz_json = config.at("hz");
		validate_hz_json(hz_json, enable_json);

		return {
			.tf=tf,
			.sensor_hz = hz_json.at("sensors").get<double>(),
			.avionics_hz = hz_json.at("avionics").get<double>(),
			.estimation_hz = hz_json.at("estimation").get<double>(),
			.guidance_hz = hz_json.at("guidance").get<double>(),
			.control_hz = hz_json.at("control").get<double>(),

			.trim_bool=enable_json.at("trim").get<bool>(),
			.avionics_bool=enable_json.at("avionics").get<bool>(),
			.control_bool=enable_json.at("control").get<bool>(),
			.estimation_bool=enable_json.at("estimation").get<bool>(),
			.wind_bool=enable_json.at("wind").get<bool>(),
			.verbose_bool=enable_json.at("verbose").get<bool>(),
			.rerun_bool=enable_json.at("rerun").get<bool>()
		};

    }

}
