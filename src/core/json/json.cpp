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
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"

namespace json {

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

    void _validate_NEDFrameECEF_initialization_config(const nlohmann::json& frame_json);
    void _validate_FRDFrameECEF_initialization_config(const nlohmann::json& frame_json);
    void _validate_FRDFrameNED_initialization_config(const nlohmann::json& frame_json);
    void _validate_WINDFrameSTAB_initialization_config(const nlohmann::json& frame_json);

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

    Eigen::Quaterniond parse_Quaterniond(const nlohmann::json& values) {
        const Eigen::Vector4d q = parse_Vector4d(values);
        return Eigen::Quaterniond(q(0), q(1), q(2), q(3));
    }

    double parse_sensor_scalar_bias(const nlohmann::json& sensor_json) {
        if (!sensor_json.contains("bias")) { return 0.0; }
        return sensor_json.at("bias").get<double>();
    }

    Eigen::Vector3d parse_sensor_3d_bias(const nlohmann::json& sensor_json) {
        if (!sensor_json.contains("bias")) { return constants::Zero3; }
        return parse_Vector3d(sensor_json.at("bias"));
    }

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

    actuators::ActuatorLimits parse_actuator_limits(const nlohmann::json& limits_json) {
        return {
            .elevator_max = util::deg_to_rad(limits_json.value("elevator_max", 0.0)),
            .aileron_max = util::deg_to_rad(limits_json.value("aileron_max", 0.0)),
            .rudder_max = util::deg_to_rad(limits_json.value("rudder_max", 0.0)),
            .flap_max = util::deg_to_rad(limits_json.value("flap_max", 0.0)),
            .spoiler_max = util::deg_to_rad(limits_json.value("spoiler_max", 0.0)),
        };
    }

    template <typename ControllerType>
    control::ControlLaw make_stateful_control_law(const control::ControlLawParameters& params) {
        return [controller = ControllerType(params)](const control::ControlLawInput& ctrl_law_input) mutable {
            return controller._step(ctrl_law_input);
        };
    }

    Eigen::VectorXd parse_control_gains(const nlohmann::json& gains_json, const std::string& control_type) {
        Eigen::VectorXd gains = Eigen::VectorXd::Zero(3);

        if (gains_json.is_array()) {
            if (gains_json.size() > 3) { throw std::runtime_error("json::parse_control_gains expected at most 3 gains"); }
            for (int i = 0; i < static_cast<int>(gains_json.size()); ++i) {
                gains[i] = gains_json.at(i).get<double>();
            }
            return gains;
        }

        if (!gains_json.is_number()) { throw std::runtime_error("json::parse_control_gains expected number or array"); }
        const double gain = gains_json.get<double>();
        gains[0] = gain;
        return gains;
    }

    control::ControlLawParameters parse_control_law_parameters(const nlohmann::json& controller_json, const std::string& control_type) {
        return {
            .gains = parse_control_gains(controller_json.at("gains"), control_type),
        };
    }

    control::ControlLaw make_control_law(const std::string& control_type, const control::ControlLawParameters& params) {
        if (control_type == "RollPIDController") { return make_stateful_control_law<control::RollPIDController>(params); }
        if (control_type == "PitchPIDController") { return make_stateful_control_law<control::PitchPIDController>(params); }
        if (control_type == "YawPIDController") { return make_stateful_control_law<control::YawPIDController>(params); }
        if (control_type == "RollDamper") { return make_stateful_control_law<control::RollDamper>(params); }
        if (control_type == "PitchDamper") { return make_stateful_control_law<control::PitchDamper>(params); }
        if (control_type == "YawDamper") { return make_stateful_control_law<control::YawDamper>(params); }
        throw std::runtime_error("json::make_control_law unknown control type: " + control_type);
    }

    vehicles::_StepOptions to_step_options(const vehicles::FRDFrameNEDStepOptions& opts) {
        return {
            .H = opts.HNB,
            .C = opts.CNB,
            .p = opts.pN_BN,
            .q = opts.qNB,
            .eul = opts.eulNB,
            .C_dot = opts.CNB_dot,
            .q_dot = opts.qNB_dot,
            .w = opts.wB_BN,
            .eul_dot = opts.eulNB_dot,
            .wq = opts.wq_BN,
            .v = opts.vB_BN,
            .rbs = opts.rbs_BN
        };
    }

    frames::SetOptions to_set_options(const vehicles::FRDFrameNEDStepOptions& opts) {
        frames::SetOptions set_opts{
            .H = opts.HNB,
            .C = opts.CNB,
            .p = opts.pN_BN,
            .q = opts.qNB,
            .eul = opts.eulNB,
            .C_dot = opts.CNB_dot,
            .q_dot = opts.qNB_dot,
            .w = opts.wB_BN,
            .eul_dot = opts.eulNB_dot,
            .wq = opts.wq_BN,
            .v = opts.vB_BN
        };

        if (opts.rbs_BN.has_value()) {
            set_opts.p = opts.rbs_BN->p;
            set_opts.q = opts.rbs_BN->q;
            set_opts.w = opts.rbs_BN->w;
            set_opts.v = opts.rbs_BN->v;
        }

        return set_opts;
    }

    dynamics::RigidBodyState rigid_body_state(const frames::Frame& F) {
        const frames::FrameView fv = F.view();
        return { .p = fv.H->p(), .v = *fv.v, .q = *fv.q, .w = *fv.w };
    }

    dynamics::RigidBodyState parse_control_target_state(const vehicles::FRDFrameNEDStepOptions& opts) {
        frames::NEDFrameECEF ned_frame_fake;
        frames::FRDFrameNED frd_frame_fake(&ned_frame_fake);
        vehicles::_StepOptions::_validate(frd_frame_fake, to_step_options(opts));
        frd_frame_fake.set(to_set_options(opts));
        return rigid_body_state(frd_frame_fake);
    }

    void _validate_initialization_config(const nlohmann::json& cfg, bool trim_bool) {
        if (!cfg.contains("NEDFrameECEF") && !cfg.contains("FRDFrameECEF")) { throw std::runtime_error("json::_validate_initialization_config: One of NEDFrameECEF, FRDFrameECEF required"); }
        if (!cfg.contains("FRDFrameECEF") && !cfg.contains("FRDFrameNED")) { throw std::runtime_error("json::_validate_initialization_config: One of FRDFrameECEF, FRDFrameNED required"); }
        if (cfg.contains("STABFrameFRD")) { throw std::runtime_error("json::_validate_initialization_config: STABFrameFRD initialization not allowed"); }
        if (!trim_bool && cfg.contains("WINDFrameSTAB")) { throw std::runtime_error("json::_validate_initialization_config: WINDFrameSTAB requires trim to be enabled"); }
        if (trim_bool && !cfg.contains("WINDFrameSTAB")) { throw std::runtime_error("json::_validate_initialization_config: trim requires WINDFrameSTAB"); }
        if (trim_bool && !cfg.contains("FRDFrameNED")) { throw std::runtime_error("json::_validate_initialization_config: FRDFrameNED required for trim"); }

        if (cfg.contains("NEDFrameECEF")) { _validate_NEDFrameECEF_initialization_config(cfg.at("NEDFrameECEF")); }
        if (cfg.contains("FRDFrameECEF")) { _validate_FRDFrameECEF_initialization_config(cfg.at("FRDFrameECEF")); }
        if (cfg.contains("FRDFrameNED")) { _validate_FRDFrameNED_initialization_config(cfg.at("FRDFrameNED")); }
        if (cfg.contains("WINDFrameSTAB")) { _validate_WINDFrameSTAB_initialization_config(cfg.at("WINDFrameSTAB")); }
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

    void parse_axis_controller(const nlohmann::json& controllers_json, const std::string& key, std::string& control_type, control::ControlLaw& controller) {
        if (!controllers_json.contains(key)) { return; }

        const auto& controller_json = controllers_json.at(key);
        control_type = controller_json.at("control_type").get<std::string>();
        controller = make_control_law(control_type, parse_control_law_parameters(controller_json, control_type));
    }

    control::ControlProperties parse_control_properties(const nlohmann::json& control_json) {
        control::ControlProperties control_properties{
            .xN_des_t = {
                .p = dynamics::Position{ constants::Zero3 },
                .v = dynamics::LinearVelocity{ constants::Zero3 },
                .q = dynamics::OrientationQuaternion{ constants::qI },
                .w = dynamics::AngularVelocity{ constants::Zero3 },
            },
        };

        if (control_json.contains("controllers")) {
            const auto& controllers_json = control_json.at("controllers");
            control_properties.full_state = controllers_json.value("full_state", false);
            parse_axis_controller(controllers_json, "longitudinal", control_properties.longitudinal_control_type, control_properties.longitudinal_controller);
            parse_axis_controller(controllers_json, "lateral", control_properties.lateral_control_type, control_properties.lateral_controller);
            parse_axis_controller(controllers_json, "vertical", control_properties.vertical_control_type, control_properties.vertical_controller);
        }

        if (control_json.contains("FRDFrameNED")) {
            _validate_FRDFrameNED_initialization_config(control_json.at("FRDFrameNED"));
            vehicles::FRDFrameNEDStepOptions target_opts = parse_FRDFrameNED_step_options(control_json.at("FRDFrameNED"));
            control_properties.xN_des_t = parse_control_target_state(target_opts);
        }

        return control_properties;
    }

    actuators::ActuatorProperties parse_actuator_properties(const nlohmann::json& actuator_json) {
        actuators::ActuatorProperties actuator_properties{};

        if (actuator_json.contains("control_surface_limits")) {
            actuator_properties.limits = parse_actuator_limits(actuator_json.at("control_surface_limits"));
        }

        return actuator_properties;
    }

    template <typename SensorType>
    SensorType parse_sensor_config(const nlohmann::json& cfg, const std::string& key) {
        const auto& sensor_json = cfg.at(key);
        const bool has_vector_bias = sensor_json.contains("bias") && sensor_json.at("bias").is_array();
        const double bias = has_vector_bias ? 0.0 : parse_sensor_scalar_bias(sensor_json);
        const Eigen::Vector3d bias_3d = has_vector_bias ? parse_sensor_3d_bias(sensor_json) : constants::Zero3;

        SensorType sensor{ avionics::Sensor{
            sensor_json.value("mean", 0.0),
            sensor_json.value("stddev", 0.0),
            bias,
            bias_3d
        } };

        sensor.tau = sensor_json.value("tau", constants::eps);
        if (sensor.tau <= constants::eps) { sensor.tau = constants::eps; }
        sensor.alpha = std::exp(-constants::dt / sensor.tau);
        return sensor;
    }

    nlohmann::json read_json_file(const std::filesystem::path& path) {
        std::ifstream file(path);
        if (!file.is_open()) { throw std::runtime_error("failed to open file: " + path.string()); }

        nlohmann::json cfg;
        file >> cfg;
        return cfg;
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

    void write_json(const nlohmann::json& cfg, const std::string& dir, const std::string& fname) {
        std::filesystem::create_directories(dir);

        const auto path_name = std::filesystem::path(dir) / (fname + ".json");
        std::ofstream file_json(path_name);
        if (!file_json.is_open()) { throw std::runtime_error("Failed to open file: " + path_name.string()); }

        file_json << cfg.dump(4) << "\n";
        file_json.close();
        std::cout << "File saved successfully to " << path_name.string() << std::endl;
    }

    void dump_configs(const std::string& dir) {
        const auto run_path = std::filesystem::path("config") / "run.json";
        const auto run_cfg = read_json_file(run_path);

        for (const auto& [key, value] : run_cfg.items()) {
            if (!value.is_string()) { throw std::runtime_error("json::dump_configs: expected string path for key '" + key + "'"); }
            const auto cfg_path = resolve_config_path(run_path, value.get<std::string>());
            write_json(read_json_file(cfg_path), dir, key);
        }
    }

    aerodynamics::AerodynamicProperties parse_aerodynamics_config() {
        const auto aerodynamics_cfg_path = resolve_run_config_entry_path("aerodynamics_config");
        const auto cfg = read_json_file(aerodynamics_cfg_path);
        const auto& surfaces_json = cfg.at("surfaces");
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

    actuators::ActuatorProperties parse_actuator_config() {
        const auto actuator_cfg_path = resolve_run_config_entry_path("actuator_config");
        return parse_actuator_properties(read_json_file(actuator_cfg_path));
    }

    control::ControlProperties parse_control_config() {
        const auto control_cfg_path = resolve_run_config_entry_path("control_config");
        return parse_control_properties(read_json_file(control_cfg_path));
    }

    vehicles::StepOptions parse_initialization_config(bool trim_bool) {
        const auto init_cfg_path = resolve_run_config_entry_path("initialization_config");
        const auto cfg = read_json_file(init_cfg_path);
        _validate_initialization_config(cfg, trim_bool);

        vehicles::StepOptions opts;
        if (cfg.contains("NEDFrameECEF")) { opts.NEDFrameECEFStepOpts = parse_NEDFrameECEF_step_options(cfg.at("NEDFrameECEF")); }
        if (cfg.contains("FRDFrameECEF")) { opts.FRDFrameECEFStepOpts = parse_FRDFrameECEF_step_options(cfg.at("FRDFrameECEF")); }
        if (cfg.contains("FRDFrameNED")) { opts.FRDFrameNEDStepOpts = parse_FRDFrameNED_step_options(cfg.at("FRDFrameNED")); }
        if (cfg.contains("STABFrameFRD")) { opts.STABFrameFRDStepOpts = parse_STABFrameFRD_step_options(cfg.at("STABFrameFRD")); }
        if (cfg.contains("WINDFrameSTAB")) { opts.WINDFrameSTABStepOpts = parse_WINDFrameSTAB_step_options(cfg.at("WINDFrameSTAB")); }
        return opts;
    }

    structural::StructuralProperties parse_structural_config() {
        const auto structural_cfg_path = resolve_run_config_entry_path("structural_config");
        const auto cfg = read_json_file(structural_cfg_path);
        const auto& geometries_json = cfg.at("geometries");
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

    avionics::AvionicsProperties parse_avionics_config() {
        const auto avionics_cfg_path = resolve_run_config_entry_path("avionics_config");
        const auto cfg = read_json_file(avionics_cfg_path);

        avionics::AvionicsSensors sensors = {
            .aoa_vane = parse_sensor_config<avionics::AngleOfAttackVane>(cfg, "AngleOfAttackVane"),
            .accelerometer = parse_sensor_config<avionics::Accelerometer>(cfg, "Accelerometer"),
            .gyro = parse_sensor_config<avionics::Gyroscope>(cfg, "Gyroscope"),
            .pitot_tube = parse_sensor_config<avionics::PitotTube>(cfg, "PitotTube"),
            .static_port = parse_sensor_config<avionics::StaticPort>(cfg, "StaticPort"),
            .tat_probe = parse_sensor_config<avionics::TotalAirTemperatureProbe>(cfg, "TotalAirTemperatureProbe"),
            .gnss = parse_sensor_config<avionics::GNSSReceiver>(cfg, "GNSSReceiver"),
            .magnetometer = parse_sensor_config<avionics::Magnetometer>(cfg, "Magnetometer"),
        };

        avionics::AvionicsComputers computers = { .ADC = {}, .AHRS = {}, .INS = {} };
        return { .sensors = sensors, .computers = computers, .hist = {}, .cache = {} };
    }

}
