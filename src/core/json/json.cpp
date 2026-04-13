#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "core/json/json.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/avionics/avionics.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/control/control.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/frames/frames.hpp"
#include "simulation/geography/geography.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/util/util.hpp"
#include "simulation/vehicles/vehicles.hpp"

namespace json {

    Eigen::Vector3d parse_Vector3d(const nlohmann::json& values) {
        if (!values.is_array() || values.size() != 3) { throw std::runtime_error("expected a 3-element array"); }
        return Eigen::Vector3d(values.at(0).get<double>(), values.at(1).get<double>(), values.at(2).get<double>());
    }

    Eigen::Vector4d parse_Vector4d(const nlohmann::json& values) {
        if (!values.is_array() || values.size() != 4) { throw std::runtime_error("expected a 4-element array"); }
        return Eigen::Vector4d(values.at(0).get<double>(), values.at(1).get<double>(), values.at(2).get<double>(), values.at(3).get<double>());
    }

    Eigen::Matrix3d parse_Matrix3d(const nlohmann::json& values) {
        if (!values.is_array() || values.size() != 3) { throw std::runtime_error("expected a 3x3 array"); }

        Eigen::Matrix3d out;
        for (int row = 0; row < 3; ++row) {
            const auto& row_values = values.at(row);
            if (!row_values.is_array() || row_values.size() != 3) { throw std::runtime_error("expected a 3x3 array"); }
            for (int col = 0; col < 3; ++col) {
                out(row, col) = row_values.at(col).get<double>();
            }
        }
        return out;
    }

    Eigen::Matrix4d parse_Matrix4d(const nlohmann::json& values) {
        if (!values.is_array() || values.size() != 4) { throw std::runtime_error("expected a 4x4 array"); }

        Eigen::Matrix4d out;
        for (int row = 0; row < 4; ++row) {
            const auto& row_values = values.at(row);
            if (!row_values.is_array() || row_values.size() != 4) { throw std::runtime_error("expected a 4x4 array"); }
            for (int col = 0; col < 4; ++col) {
                out(row, col) = row_values.at(col).get<double>();
            }
        }
        return out;
    }

    Eigen::MatrixXd parse_MatrixXd(const nlohmann::json& values) {
        if (!values.is_array()) { throw std::runtime_error("expected matrix array"); }
        if (values.empty()) { return Eigen::MatrixXd{}; }

        const int rows = static_cast<int>(values.size());
        const int cols = static_cast<int>(values.at(0).size());
        Eigen::MatrixXd out(rows, cols);

        for (int row = 0; row < rows; ++row) {
            const auto& row_values = values.at(row);
            if (!row_values.is_array() || static_cast<int>(row_values.size()) != cols) { throw std::runtime_error("expected rectangular matrix"); }
            for (int col = 0; col < cols; ++col) {
                out(row, col) = row_values.at(col).get<double>();
            }
        }
        return out;
    }

    Eigen::Quaterniond parse_Quaterniond(const nlohmann::json& values) {
        const Eigen::Vector4d q = parse_Vector4d(values);
        return Eigen::Quaterniond(q(0), q(1), q(2), q(3));
    }

    nlohmann::json read_json_file(const std::filesystem::path& path) {
        std::ifstream file(path);
        if (!file.is_open()) { throw std::runtime_error("failed to open file: " + path.string()); }

        nlohmann::json config;
        file >> config;
        return config;
    }

    std::filesystem::path resolve_config_path(const std::filesystem::path& run_path, const std::string& config_path) {
        const std::filesystem::path path{ config_path };
        if (path.is_absolute()) { return path; }
        return run_path.parent_path() / path;
    }

    std::filesystem::path resolve_run_config_entry_path(const std::string& key) {
        const auto run_path = std::filesystem::path("config") / "run.json";
        const auto run_config = read_json_file(run_path);
        return resolve_config_path(run_path, run_config.at(key).get<std::string>());
    }

    void write_json(const nlohmann::json& config, const std::string& dir, const std::string& fname) {
        std::filesystem::create_directories(dir);

        const auto path_name = std::filesystem::path(dir) / (fname + ".json");
        std::ofstream file_json(path_name);
        if (!file_json.is_open()) { throw std::runtime_error("Failed to open file: " + path_name.string()); }

        file_json << config.dump(4) << "\n";
        file_json.close();
        std::cout << "File saved successfully to " << path_name.string() << std::endl;
    }

    void dump_configs(const std::string& dir) {
        const auto run_path = std::filesystem::path("config") / "run.json";
        const auto run_config = read_json_file(run_path);

        for (const auto& [key, value] : run_config.items()) {
            if (!value.is_string()) { throw std::runtime_error("json::dump_configs: expected string path for key '" + key + "'"); }
            const auto config_path = resolve_config_path(run_path, value.get<std::string>());
            write_json(read_json_file(config_path), dir, key);
        }
    }

    aerodynamics::DynamicDerivatives parse_dynamic_derivatives(const nlohmann::json& dyn_json) {
        return {
            .CL_qhat = dyn_json.value("CL_qhat", 0.0),
            .CD_qhat = dyn_json.value("CD_qhat", 0.0),
            .CM_qhat = dyn_json.value("CM_qhat", 0.0),
            .CL_phat = dyn_json.value("CL_phat", 0.0),
            .CD_phat = dyn_json.value("CD_phat", 0.0),
            .CM_phat = dyn_json.value("CM_phat", 0.0),
            .CL_rhat = dyn_json.value("CL_rhat", 0.0),
            .CD_rhat = dyn_json.value("CD_rhat", 0.0),
            .CM_rhat = dyn_json.value("CM_rhat", 0.0),
        };
    }

    aerodynamics::ControlDerivatives parse_control_derivatives(const nlohmann::json& ctrl_json) {
        return {
            .dCL_de = ctrl_json.value("dCL_de", 0.0),
            .dCM_de = ctrl_json.value("dCM_de", 0.0),
            .dCD_de = ctrl_json.value("dCD_de", 0.0),
            .dCL_da = ctrl_json.value("dCL_da", 0.0),
            .dCM_da = ctrl_json.value("dCM_da", 0.0),
            .dCD_da = ctrl_json.value("dCD_da", 0.0),
            .dCL_dr = ctrl_json.value("dCL_dr", 0.0),
            .dCM_dr = ctrl_json.value("dCM_dr", 0.0),
            .dCD_dr = ctrl_json.value("dCD_dr", 0.0),
            .dCL_df = ctrl_json.value("dCL_df", 0.0),
            .dCM_df = ctrl_json.value("dCM_df", 0.0),
            .dCD_df = ctrl_json.value("dCD_df", 0.0),
            .dCL_ds = ctrl_json.value("dCL_ds", 0.0),
            .dCM_ds = ctrl_json.value("dCM_ds", 0.0),
            .dCD_ds = ctrl_json.value("dCD_ds", 0.0),
        };
    }
    aerodynamics::AerodynamicProperties parse_aerodynamic_properties(const nlohmann::json& config) {
        const auto& surfaces_json = config.at("surfaces");
        if (!surfaces_json.is_array()) { throw std::runtime_error("json::parse_aerodynamics_config expected 'surfaces' to be an array"); }

        std::vector<aerodynamics::Surface> surfaces;
        surfaces.reserve(surfaces_json.size());
        for (const auto& surface_json : surfaces_json) {
            surfaces.push_back(aerodynamics::Surface{
                .id = surface_json.at("id").get<std::string>(),
                .chord = surface_json.at("chord").get<double>(),
                .span = surface_json.at("span").get<double>(),
                .p_ref = parse_Vector3d(surface_json.at("p_ref")),
                .n = parse_Vector3d(surface_json.at("n")),
                .CL0 = surface_json.at("CL0").get<double>(),
                .e = surface_json.at("e").get<double>(),
                .i = surface_json.at("i").get<double>(),
                .CD0 = surface_json.at("CD0").get<double>(),
                .CDa = surface_json.at("CDa").get<double>(),
                .a0 = surface_json.at("a0").get<double>(),
                .CM0 = surface_json.at("CM0").get<double>(),
                .CMa = surface_json.at("CMa").get<double>(),
                .dyn = surface_json.contains("dyn") ? parse_dynamic_derivatives(surface_json.at("dyn")) : aerodynamics::DynamicDerivatives{},
                .ctrl = surface_json.contains("ctrl") ? parse_control_derivatives(surface_json.at("ctrl")) : aerodynamics::ControlDerivatives{},
            });
        }
        return { surfaces };
    }

    aerodynamics::AerodynamicProperties parse_aerodynamics_config() {
        const auto config_path = resolve_run_config_entry_path("aerodynamics_config");
        const auto config = read_json_file(config_path);
        return parse_aerodynamic_properties(config);
    }

    double parse_sensor_scalar_bias(const nlohmann::json& sensor_json) {
        return sensor_json.at("bias").get<double>();
    }

    Eigen::Vector3d parse_sensor_3d_bias(const nlohmann::json& sensor_json) {
        return parse_Vector3d(sensor_json.at("bias"));
    }

    template <typename SensorType>
    SensorType parse_sensor(const nlohmann::json& config, const std::string& key) {
        const auto& sensor_json = config.at(key);
        const bool has_vector_bias = sensor_json.at("bias").is_array();
        const double bias = has_vector_bias ? 0.0 : parse_sensor_scalar_bias(sensor_json);
        const Eigen::Vector3d bias_3d = has_vector_bias ? parse_sensor_3d_bias(sensor_json) : constants::Zero3;

        SensorType sensor{ avionics::Sensor(
            sensor_json.value("mean", 0.0),
            sensor_json.value("stddev", 0.0),
            bias,
            bias_3d,
            sensor_json.value("tau", constants::eps)
        ) };

        return sensor;
    }

    avionics::AvionicsProperties parse_avionics_properties(const nlohmann::json& config) {
        avionics::AvionicsSensors sensors = {
            .aoa_vane = parse_sensor<avionics::AngleOfAttackVane>(config, "angle_of_attack_vane"),
            .accelerometer = parse_sensor<avionics::Accelerometer>(config, "accelerometer"),
            .gyro = parse_sensor<avionics::Gyroscope>(config, "gyroscope"),
            .pitot_tube = parse_sensor<avionics::PitotTube>(config, "pitot_tube"),
            .static_port = parse_sensor<avionics::StaticPort>(config, "static_port"),
            .tat_probe = parse_sensor<avionics::TotalAirTemperatureProbe>(config, "total_air_temperature_probe"),
            .gnss = parse_sensor<avionics::GNSSReceiver>(config, "gnss_receiver"),
            .magnetometer = parse_sensor<avionics::Magnetometer>(config, "magnetometer"),
        };

        return { .sensors = sensors };
    }


    avionics::AvionicsProperties parse_avionics_config() {
        const auto config_path = resolve_run_config_entry_path("avionics_config");
        const auto config = read_json_file(config_path);
        return parse_avionics_properties(config);
    }

    template <typename ActuatorType>
    ActuatorType parse_actuator(const nlohmann::json& config, const std::string& key) {
        const auto& actuator_json = config.at(key);

        ActuatorType actuator{ actuators::Actuator{
            .limit_max = util::deg_to_rad(actuator_json.value("limit_max", 0.0)),
            .limit_min = util::deg_to_rad(actuator_json.value("limit_min", 0.0)),
            .tau = actuator_json.value("tau", constants::eps),
        } };

        return actuator;
    }

    actuators::ActuatorProperties parse_actuator_properties(const nlohmann::json& config) {
        actuators::Actuators actuators = {
            .elevator = parse_actuator<actuators::Elevator>(config, "elevator"),
            .aileron = parse_actuator<actuators::Aileron>(config, "aileron"),
            .rudder = parse_actuator<actuators::Rudder>(config, "rudder"),
            .flaps = parse_actuator<actuators::Flap>(config, "flaps"),
            .spoilers = parse_actuator<actuators::Spoiler>(config, "spoilers")
        };

        return { .actuators = actuators };
    }

    actuators::ActuatorProperties parse_actuator_config() {
        const auto config_path = resolve_run_config_entry_path("actuator_config");
        const auto config = read_json_file(config_path);
        return parse_actuator_properties(config);
    }

    struct ParsedStepOptions {
        std::optional<dynamics::HomogenousFrameTransformationMatrix> H;
        std::optional<dynamics::OrientationMatrix> C;
        std::optional<dynamics::Position> p;
        std::optional<dynamics::OrientationQuaternion> q;
        std::optional<dynamics::EulerAngles> eul;
        std::optional<dynamics::OrientationMatrixRate> C_dot;
        std::optional<dynamics::OrientationQuaternionRate> q_dot;
        std::optional<dynamics::AngularVelocity> w;
        std::optional<dynamics::EulerAngleRates> eul_dot;
        std::optional<dynamics::AngularVelocityQuaternion> wq;
        std::optional<dynamics::LinearVelocity> v;
        std::optional<geography::Latitude> lat;
        std::optional<geography::Longitude> lon;
        std::optional<geography::Altitude> alt;
        std::optional<aerodynamics::AngleOfAttack> alpha;
        std::optional<aerodynamics::SideslipAngle> beta;
    };

    ParsedStepOptions parse_step_options(const nlohmann::json& frame_json) {
        ParsedStepOptions fields;

        if (frame_json.contains("H")) { fields.H = dynamics::HomogenousFrameTransformationMatrix{ parse_Matrix4d(frame_json.at("H")) }; }
        if (frame_json.contains("C")) { fields.C = dynamics::OrientationMatrix{ parse_Matrix3d(frame_json.at("C")) }; }
        if (frame_json.contains("p")) { fields.p = dynamics::Position{ parse_Vector3d(frame_json.at("p")) }; }
        if (frame_json.contains("q")) { fields.q = dynamics::OrientationQuaternion{ parse_Quaterniond(frame_json.at("q")) }; }
        if (frame_json.contains("eul")) { fields.eul = dynamics::EulerAngles{ parse_Vector3d(frame_json.at("eul")) }; }
        if (frame_json.contains("C_dot")) { fields.C_dot = dynamics::OrientationMatrixRate{ parse_Matrix3d(frame_json.at("C_dot")) }; }
        if (frame_json.contains("q_dot")) { fields.q_dot = dynamics::OrientationQuaternionRate{ parse_Quaterniond(frame_json.at("q_dot")) }; }
        if (frame_json.contains("w")) { fields.w = dynamics::AngularVelocity{ parse_Vector3d(frame_json.at("w")) }; }
        if (frame_json.contains("eul_dot")) { fields.eul_dot = dynamics::EulerAngleRates{ parse_Vector3d(frame_json.at("eul_dot")) }; }
        if (frame_json.contains("wq")) { fields.wq = dynamics::AngularVelocityQuaternion{ parse_Quaterniond(frame_json.at("wq")) }; }
        if (frame_json.contains("v")) { fields.v = dynamics::LinearVelocity{ parse_Vector3d(frame_json.at("v")) }; }
        if (frame_json.contains("lat")) { fields.lat = geography::Latitude{ util::deg_to_rad(frame_json.at("lat").get<double>()) }; }
        if (frame_json.contains("lon")) { fields.lon = geography::Longitude{ util::deg_to_rad(frame_json.at("lon").get<double>()) }; }
        if (frame_json.contains("alt")) { fields.alt = geography::Altitude{ frame_json.at("alt").get<double>() }; }
        if (frame_json.contains("alpha")) { fields.alpha = aerodynamics::AngleOfAttack{ frame_json.at("alpha").get<double>() }; }
        if (frame_json.contains("beta")) { fields.beta = aerodynamics::SideslipAngle{ frame_json.at("beta").get<double>() }; }

        return fields;
    }

    vehicles::NEDFrameECEFStepOptions parse_NEDFrameECEF_step_options(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        return { .lat_NE = fields.lat, .lon_NE = fields.lon, .alt_NE = fields.alt };
    }

    vehicles::FRDFrameECEFStepOptions parse_FRDFrameECEF_step_options(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        return {
            .HEB = fields.H,
            .CEB = fields.C,
            .pE_BE = fields.p,
            .qEB = fields.q,
            .eulEB = fields.eul,
            .CEB_dot = fields.C_dot,
            .qEB_dot = fields.q_dot,
            .wB_BE = fields.w,
            .eulEB_dot = fields.eul_dot,
            .wq_BE = fields.wq,
            .vB_BE = fields.v,
            .lat_BE = fields.lat,
            .lon_BE = fields.lon,
            .alt_BE = fields.alt,
        };
    }

    vehicles::FRDFrameNEDStepOptions parse_FRDFrameNED_step_options(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        return {
            .HNB = fields.H,
            .CNB = fields.C,
            .pN_BN = fields.p,
            .qNB = fields.q,
            .eulNB = fields.eul,
            .CNB_dot = fields.C_dot,
            .qNB_dot = fields.q_dot,
            .wB_BN = fields.w,
            .eulNB_dot = fields.eul_dot,
            .wq_BN = fields.wq,
            .vB_BN = fields.v,
        };
    }

    vehicles::STABFrameFRDStepOptions parse_STABFrameFRD_step_options(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        return { .alpha = fields.alpha };
    }

    vehicles::WINDFrameSTABStepOptions parse_WINDFrameSTAB_step_options(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        return { .beta = fields.beta };
    }

    void _validate_NEDFrameECEF_initialization_config(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        if (!fields.lat.has_value() || !fields.lon.has_value() || !fields.alt.has_value()) { throw std::runtime_error("json::_validate_NEDFrameECEF_initialization_config: lat, lon, alt required"); }
    }

    void _validate_FRDFrameECEF_initialization_config(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        const bool has_H = fields.H.has_value();
        const bool has_C = fields.C.has_value();
        const bool has_p = fields.p.has_value();
        const bool has_q = fields.q.has_value();
        const bool has_eul = fields.eul.has_value();
        const bool has_C_dot = fields.C_dot.has_value();
        const bool has_q_dot = fields.q_dot.has_value();
        const bool has_w = fields.w.has_value();
        const bool has_eul_dot = fields.eul_dot.has_value();
        const bool has_wq = fields.wq.has_value();
        const bool has_v = fields.v.has_value();
        const bool has_lat = fields.lat.has_value();
        const bool has_lon = fields.lon.has_value();
        const bool has_alt = fields.alt.has_value();

        const bool has_geo_all = has_lat && has_lon && has_alt;
        const bool has_position = has_H || has_p || has_geo_all;
        const bool has_orientation = has_H || has_C || has_q || has_eul;
        const bool has_linear_velocity = has_v;
        const bool has_angular_velocity = has_C_dot || has_q_dot || has_w || has_eul_dot || has_wq;

        if (!has_position) { throw std::runtime_error("json::_validate_FRDFrameECEF_initialization_config: one position representation required"); }
        if (!has_orientation) { throw std::runtime_error("json::_validate_FRDFrameECEF_initialization_config: one orientation representation required"); }
        if (!has_linear_velocity) { throw std::runtime_error("json::_validate_FRDFrameECEF_initialization_config: v required"); }
        if (!has_angular_velocity) { throw std::runtime_error("json::_validate_FRDFrameECEF_initialization_config: one angular velocity representation required"); }
    }

    void _validate_FRDFrameNED_initialization_config(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        const bool has_H = fields.H.has_value();
        const bool has_C = fields.C.has_value();
        const bool has_p = fields.p.has_value();
        const bool has_q = fields.q.has_value();
        const bool has_eul = fields.eul.has_value();
        const bool has_C_dot = fields.C_dot.has_value();
        const bool has_q_dot = fields.q_dot.has_value();
        const bool has_w = fields.w.has_value();
        const bool has_eul_dot = fields.eul_dot.has_value();
        const bool has_wq = fields.wq.has_value();
        const bool has_v = fields.v.has_value();

        const bool has_position = has_H || has_p;
        const bool has_orientation = has_H || has_C || has_q || has_eul;
        const bool has_linear_velocity = has_v;
        const bool has_angular_velocity = has_C_dot || has_q_dot || has_w || has_eul_dot || has_wq;

        if (!has_position) { throw std::runtime_error("json::_validate_FRDFrameNED_initialization_config: one position representation required"); }
        if (!has_orientation) { throw std::runtime_error("json::_validate_FRDFrameNED_initialization_config: one orientation representation required"); }
        if (!has_linear_velocity) { throw std::runtime_error("json::_validate_FRDFrameNED_initialization_config: v required"); }
        if (!has_angular_velocity) { throw std::runtime_error("json::_validate_FRDFrameNED_initialization_config: one angular velocity representation required"); }
    }

    void _validate_WINDFrameSTAB_initialization_config(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        if (!fields.beta.has_value()) { throw std::runtime_error("json::_validate_WINDFrameSTAB_initialization_config: beta required"); }
    }

    void _validate_initialization_config(const nlohmann::json& config, bool trim_bool) {
        if (!config.contains("NEDFrameECEF") && !config.contains("FRDFrameECEF")) { throw std::runtime_error("json::_validate_initialization_config: One of NEDFrameECEF, FRDFrameECEF required"); }
        if (!config.contains("FRDFrameECEF") && !config.contains("FRDFrameNED")) { throw std::runtime_error("json::_validate_initialization_config: One of FRDFrameECEF, FRDFrameNED required"); }
        if (config.contains("STABFrameFRD")) { throw std::runtime_error("json::_validate_initialization_config: STABFrameFRD initialization not allowed"); }
        if (!trim_bool && config.contains("WINDFrameSTAB")) { throw std::runtime_error("json::_validate_initialization_config: WINDFrameSTAB requires trim to be enabled"); }
        if (trim_bool && !config.contains("WINDFrameSTAB")) { throw std::runtime_error("json::_validate_initialization_config: trim requires WINDFrameSTAB"); }
        if (trim_bool && !config.contains("FRDFrameNED")) { throw std::runtime_error("json::_validate_initialization_config: FRDFrameNED required for trim"); }

        if (config.contains("NEDFrameECEF")) { _validate_NEDFrameECEF_initialization_config(config.at("NEDFrameECEF")); }
        if (config.contains("FRDFrameECEF")) { _validate_FRDFrameECEF_initialization_config(config.at("FRDFrameECEF")); }
        if (config.contains("FRDFrameNED")) { _validate_FRDFrameNED_initialization_config(config.at("FRDFrameNED")); }
        if (config.contains("WINDFrameSTAB")) { _validate_WINDFrameSTAB_initialization_config(config.at("WINDFrameSTAB")); }
    }

    vehicles::StepOptions parse_initialization_config(bool trim_bool) {
        const auto config_path = resolve_run_config_entry_path("initialization_config");
        const auto config = read_json_file(config_path);
        _validate_initialization_config(config, trim_bool);

        vehicles::StepOptions opts;
        if (config.contains("NEDFrameECEF")) { opts.NEDFrameECEFStepOpts = parse_NEDFrameECEF_step_options(config.at("NEDFrameECEF")); }
        if (config.contains("FRDFrameECEF")) { opts.FRDFrameECEFStepOpts = parse_FRDFrameECEF_step_options(config.at("FRDFrameECEF")); }
        if (config.contains("FRDFrameNED")) { opts.FRDFrameNEDStepOpts = parse_FRDFrameNED_step_options(config.at("FRDFrameNED")); }
        if (config.contains("STABFrameFRD")) { opts.STABFrameFRDStepOpts = parse_STABFrameFRD_step_options(config.at("STABFrameFRD")); }
        if (config.contains("WINDFrameSTAB")) { opts.WINDFrameSTABStepOpts = parse_WINDFrameSTAB_step_options(config.at("WINDFrameSTAB")); }
        return opts;
    }

    /** @deprecated */
    // vehicles::_StepOptions to_step_options(const vehicles::FRDFrameNEDStepOptions& opts) {
    //     return {
    //         .H = opts.HNB,
    //         .C = opts.CNB,
    //         .p = opts.pN_BN,
    //         .q = opts.qNB,
    //         .eul = opts.eulNB,
    //         .C_dot = opts.CNB_dot,
    //         .q_dot = opts.qNB_dot,
    //         .w = opts.wB_BN,
    //         .eul_dot = opts.eulNB_dot,
    //         .wq = opts.wq_BN,
    //         .v = opts.vB_BN,
    //         .rbs = opts.rbs_BN
    //     };
    // }

    /** @deprecated */
    // frames::SetOptions to_set_options(const vehicles::FRDFrameNEDStepOptions& opts) {
    //     frames::SetOptions set_opts{
    //         .H = opts.HNB,
    //         .C = opts.CNB,
    //         .p = opts.pN_BN,
    //         .q = opts.qNB,
    //         .eul = opts.eulNB,
    //         .C_dot = opts.CNB_dot,
    //         .q_dot = opts.qNB_dot,
    //         .w = opts.wB_BN,
    //         .eul_dot = opts.eulNB_dot,
    //         .wq = opts.wq_BN,
    //         .v = opts.vB_BN
    //     };

    //     if (opts.rbs_BN.has_value()) {
    //         set_opts.p = opts.rbs_BN->p;
    //         set_opts.q = opts.rbs_BN->q;
    //         set_opts.w = opts.rbs_BN->w;
    //         set_opts.v = opts.rbs_BN->v;
    //     }

    //     return set_opts;
    // }

    /** @deprecated */
    // dynamics::RigidBodyState rigid_body_state(const frames::Frame& F) {
    //     const frames::FrameView fv = F.view();
    //     return { .p = fv.H->p(), .v = *fv.v, .q = *fv.q, .w = *fv.w };
    // }

    /** @deprecated */
    // dynamics::RigidBodyState parse_control_target_state(const vehicles::FRDFrameNEDStepOptions& opts) {
    //     frames::NEDFrameECEF ned_frame_fake;
    //     frames::FRDFrameNED frd_frame_fake(&ned_frame_fake);
    //     vehicles::_StepOptions::_validate(frd_frame_fake, to_step_options(opts));
    //     frd_frame_fake.set(to_set_options(opts));
    //     return rigid_body_state(frd_frame_fake);
    // }

    control::AxisControlSetpoint parse_axis_control_setpoint(const nlohmann::json& setpoint_json, const control::ControlType& longitudinal_control_type, const control::ControlType& lateral_control_type, const control::ControlType& vertical_control_type) {
        control::AxisControlSetpoint axis_ctrl_setpoint;

        if (longitudinal_control_type == control::ControlType::PitchDamper || lateral_control_type == control::ControlType::RollDamper || vertical_control_type == control::ControlType::YawDamper) {
            if (setpoint_json.contains("w")) { axis_ctrl_setpoint.wB_BI = dynamics::AngularVelocity{ parse_Vector3d(setpoint_json.at("w")) }; }
            else { throw std::runtime_error("json::parse_axis_control_setpoint: Damper controller requires w"); }
        }

        if (longitudinal_control_type == control::ControlType::PitchPIDController || lateral_control_type == control::ControlType::RollPIDController || vertical_control_type == control::ControlType::YawPIDController) {
            if (setpoint_json.contains("eul")) { axis_ctrl_setpoint.eulIB = dynamics::EulerAngles{ parse_Vector3d(setpoint_json.at("eul")) }; }
            else { throw std::runtime_error("json::parse_axis_control_setpoint: PIDController requires eul"); }
        }
        return axis_ctrl_setpoint;
    }

    control::FullStateControlSetpoint parse_full_state_control_setpoint(const nlohmann::json& setpoint_json, const control::ControlType& full_state_control_type) {
        control::FullStateControlSetpoint full_state_ctrl_setpoint;

        if (full_state_control_type == control::ControlType::LinearQuadraticRegulator || full_state_control_type == control::ControlType::LinearQuadraticTracker) {
            if (setpoint_json.contains("v")) { full_state_ctrl_setpoint.vB_BI = dynamics::LinearVelocity{ parse_Vector3d(setpoint_json.at("v")) }; }
            else { throw std::runtime_error("json::parse_full_state_control_setpoint: LinearQuadratic controller requires v"); }

            if (setpoint_json.contains("w")) { full_state_ctrl_setpoint.wB_BI = dynamics::AngularVelocity{ parse_Vector3d(setpoint_json.at("w")) }; }
            else { throw std::runtime_error("json::parse_full_state_control_setpoint: LinearQuadratic controller requires w"); }

            if (setpoint_json.contains("eul")) { full_state_ctrl_setpoint.eulIB = dynamics::EulerAngles{ parse_Vector3d(setpoint_json.at("eul")) }; }
            else { throw std::runtime_error("json::parse_full_state_control_setpoint: LinearQuadratic controller requires eul"); }
        }
        return full_state_ctrl_setpoint;
    }

    void validate_control_law_tau(const nlohmann::json& parameters_json, const control::ControlType& control_type) {
        switch (control_type) {
            case control::ControlType::PitchPIDController:
            case control::ControlType::RollPIDController:
            case control::ControlType::YawPIDController:
                if (!parameters_json.contains("tau")) {
                    throw std::runtime_error("json::parse_control_law_parameters missing tau");
                }
                return;
            default:
                return;
        }
    }

    void validate_control_law_gains(const nlohmann::json& gains_json, control::ControlType& control_type) {
        if (!gains_json.is_object()) { throw std::runtime_error("json::parse_control_gains expected gains object"); }

        switch (control_type) {
            case control::ControlType::PitchPIDController:
            case control::ControlType::RollPIDController:
            case control::ControlType::YawPIDController:
                if (!gains_json.contains("Kp") || !gains_json.contains("Kd") || !gains_json.contains("Ki")) {
                    throw std::runtime_error("json::parse_control_gains PID control requires Kp, Kd, Ki");
                }
                break;

            case control::ControlType::PitchDamper:
            case control::ControlType::RollDamper:
            case control::ControlType::YawDamper:
                if (!gains_json.contains("Kp")) {
                    throw std::runtime_error("json::parse_control_gains damper control requires Kp");
                }
                break;

            case control::ControlType::LinearQuadraticRegulator:
            case control::ControlType::LinearQuadraticTracker:
                if (!gains_json.contains("Q") || !gains_json.contains("R")) {
                    throw std::runtime_error("json::parse_control_gains LQ control requires Q and R");
                }
                break;
            default:
                break;
        }
    }

    void validate_control_law_parameters(const nlohmann::json& parameters_json, const nlohmann::json& gains_json, control::ControlType& control_type) {
        validate_control_law_tau(parameters_json, control_type);
        validate_control_law_gains(gains_json, control_type);
    }

    control::ControlLawGains parse_control_gains(const nlohmann::json& gains_json, control::ControlType control_type) {
        switch (control_type) {
            case control::ControlType::PitchPIDController:
            case control::ControlType::RollPIDController:
            case control::ControlType::YawPIDController:
                return {
                    .Kp = gains_json.at("Kp").get<double>(),
                    .Kd = gains_json.at("Kd").get<double>(),
                    .Ki = gains_json.at("Ki").get<double>(),
                };

            case control::ControlType::PitchDamper:
            case control::ControlType::RollDamper:
            case control::ControlType::YawDamper:
                return {
                    .Kp = gains_json.at("Kp").get<double>(),
                };

            case control::ControlType::LinearQuadraticRegulator:
            case control::ControlType::LinearQuadraticTracker:
                return {
                    .Q = parse_MatrixXd(gains_json.at("Q")),
                    .R = parse_MatrixXd(gains_json.at("R")),
                };

            default:
                throw std::runtime_error("json::parse_control_gains unknown control type");
        }
    }

    template <typename ControlType, typename ControlLawCommand>
    control::AxisControlLaw<ControlLawCommand> make_stateful_axis_control_law(const control::ControlLawParameters& params) {
        return [controller = ControlType(params)](const control::AxisControlLawInput& ctrl_law_input) mutable {
            return controller._step(ctrl_law_input);
        };
    }

    template <typename ControlType, typename ControlLawCommand>
    control::FullStateControlLaw<ControlLawCommand> make_stateful_full_state_control_law(const control::ControlLawParameters& params) {
        return [controller = ControlType(params)](const control::FullStateControlLawInput& ctrl_law_input) mutable {
            return controller._step(ctrl_law_input);
        };
    }

    template <typename ControlLawCommand>
    control::AxisControlLaw<ControlLawCommand> make_axis_control_law(control::ControlType control_type, const control::ControlLawParameters& params) {
        switch (control_type) {
            case control::ControlType::PitchPIDController:
                return make_stateful_axis_control_law<control::PitchPIDController, double>(params);

            case control::ControlType::PitchDamper:
                return make_stateful_axis_control_law<control::PitchDamper, double>(params);

            case control::ControlType::RollPIDController:
                return make_stateful_axis_control_law<control::RollPIDController, double>(params);

            case control::ControlType::RollDamper:
                return make_stateful_axis_control_law<control::RollDamper, double>(params);

            case control::ControlType::YawPIDController:
                return make_stateful_axis_control_law<control::YawPIDController, double>(params);

            case control::ControlType::YawDamper:
                return make_stateful_axis_control_law<control::YawDamper, double>(params);

            default:
                throw std::runtime_error("json::make_axis_control_law unknown control type");
        }
    }

    template <typename ControlLawCommand>
    control::FullStateControlLaw<ControlLawCommand> make_full_state_control_law(control::ControlType control_type, const control::ControlLawParameters& params) {
        switch (control_type) {
            case control::ControlType::LinearQuadraticRegulator:
                return make_stateful_full_state_control_law<control::LinearQuadraticRegulator, Eigen::VectorXd>(params);

            case control::ControlType::LinearQuadraticTracker:
                return make_stateful_full_state_control_law<control::LinearQuadraticTracker, Eigen::VectorXd>(params);
            
            default:
                throw std::runtime_error("json::make_full_state_control_law unknown control type");
        }
    }

    control::ControlLawParameters parse_control_law_parameters(const nlohmann::json& controller_json, control::ControlType& control_type) {
        const auto& parameters_json = controller_json.at("parameters");
        const auto& gains_json = parameters_json.at("gains");

        validate_control_law_parameters(parameters_json, gains_json, control_type);

        control::ControlLawGains gains = parse_control_gains(gains_json, control_type);
        double tau = parameters_json.value("tau", constants::eps);

        return { .gains = gains, .tau = tau };
    }

    control::ControlType map_control_type(const std::string& control_type_str) {
        if (control_type_str == "PitchPIDController") { return control::ControlType::PitchPIDController; }
        if (control_type_str == "PitchDamper") { return control::ControlType::PitchDamper; }
        if (control_type_str == "RollPIDController") { return control::ControlType::RollPIDController; }
        if (control_type_str == "RollDamper") { return control::ControlType::RollDamper; }
        if (control_type_str == "YawPIDController") { return control::ControlType::YawPIDController; }
        if (control_type_str == "YawDamper") { return control::ControlType::YawDamper; }
        if (control_type_str == "LinearQuadraticRegulator") { return control::ControlType::LinearQuadraticRegulator; }
        if (control_type_str == "LinearQuadraticTracker") { return control::ControlType::LinearQuadraticTracker; }
        throw std::runtime_error("json::map_control_type unknown control type: " + control_type_str);
    }

    void parse_axis_controller(const nlohmann::json& controllers_json, const std::string& key, control::ControlType& control_type, control::AxisControlLaw<double>& control_law) {
        if (!controllers_json.contains(key)) { return; }
        const auto& controller_json = controllers_json.at(key);
        std::string control_type_str = controller_json.at("control_type").get<std::string>();

        control_type = map_control_type(control_type_str);
        control::ControlLawParameters control_law_parameters = parse_control_law_parameters(controller_json, control_type);
        control_law = make_axis_control_law<double>(control_type, control_law_parameters); 
    }

    void parse_full_state_controller(const nlohmann::json& controller_json, control::ControlType& control_type, control::FullStateControlLaw<Eigen::VectorXd>& control_law) {
        std::string control_type_str = controller_json.at("control_type").get<std::string>();

        control_type = map_control_type(control_type_str);
        control::ControlLawParameters control_law_parameters = parse_control_law_parameters(controller_json, control_type);
        control_law = make_full_state_control_law<Eigen::VectorXd>(control_type, control_law_parameters); 
    }
 
    control::ControlProperties parse_control_properties(const nlohmann::json& config) {
        const auto& controllers_json = config.at("controllers");
        bool axial_bool = controllers_json.contains("axial");
        bool full_state_bool = controllers_json.contains("full_state");
        if (axial_bool && full_state_bool) { throw std::runtime_error("json::parse_control_properties: axial and full state control laws cannot both be present"); }

        const auto& setpoint_json = config.at("setpoint");
        control::ControlProperties control_properties;

        if (axial_bool) {
            const auto& axial_controllers_json = controllers_json.at("axial");
            parse_axis_controller(
                axial_controllers_json, 
                "longitudinal", 
                control_properties.longitudinal_control_type, 
                control_properties.longitudinal_controller
            );
            parse_axis_controller(
                axial_controllers_json, 
                "lateral", 
                control_properties.lateral_control_type, 
                control_properties.lateral_controller
            );
            parse_axis_controller(
                axial_controllers_json, 
                "vertical", 
                control_properties.vertical_control_type, 
                control_properties.vertical_controller
            );

            control_properties.axis_setpoint = parse_axis_control_setpoint(
                setpoint_json, 
                control_properties.longitudinal_control_type, 
                control_properties.lateral_control_type, 
                control_properties.vertical_control_type
            );
        }

        if (full_state_bool) {
            const auto& full_state_controller_json = controllers_json.at("full_state");
            parse_full_state_controller(
                full_state_controller_json, 
                control_properties.full_state_control_type, 
                control_properties.full_state_controller
            );
            control_properties.full_state_setpoint = parse_full_state_control_setpoint(setpoint_json, control_properties.full_state_control_type);
        }

        return control_properties;
    }

    control::ControlProperties parse_control_config() {
        const auto config_path = resolve_run_config_entry_path("control_config");
        const auto config = read_json_file(config_path);
        return parse_control_properties(config);
    }

    structural::StructuralProperties parse_structural_properties(const nlohmann::json& config) {
        const auto& geometries_json = config.at("geometries");
        if (!geometries_json.is_array()) { throw std::runtime_error("json::parse_structural_config expected 'geometries' to be an array"); }

        std::vector<structural::Geometry> geometries;
        geometries.reserve(geometries_json.size());
        for (const auto& geom_json : geometries_json) {
            geometries.push_back(structural::Geometry{
                .id = geom_json.at("id").get<std::string>(),
                .mass = geom_json.at("mass").get<double>(),
                .x_size = geom_json.at("x_size").get<double>(),
                .y_size = geom_json.at("y_size").get<double>(),
                .z_size = geom_json.at("z_size").get<double>(),
                .x_loc = geom_json.at("x_loc").get<double>(),
                .y_loc = geom_json.at("y_loc").get<double>(),
                .z_loc = geom_json.at("z_loc").get<double>(),
            });
        }

        return { geometries };
    }

    structural::StructuralProperties parse_structural_config() {
        const auto config_path = resolve_run_config_entry_path("structural_config");
        const auto config = read_json_file(config_path);
        return parse_structural_properties(config);
    }

}
