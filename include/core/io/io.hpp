#pragma once
#include <filesystem>
#include <optional>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <nlohmann/json.hpp>
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/control/control.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/vehicles/vehicles.hpp"
#include "simulation/avionics/avionics.hpp"

namespace io {

    Eigen::Vector3d parse_Vector3d(const nlohmann::json& values);
    Eigen::Vector4d parse_Vector4d(const nlohmann::json& values);
    Eigen::Matrix3d parse_Matrix3d(const nlohmann::json& values);
    Eigen::Matrix4d parse_Matrix4d(const nlohmann::json& values);
    Eigen::Quaterniond parse_Quaterniond(const nlohmann::json& values);
    double parse_sensor_scalar_bias(const nlohmann::json& sensor_json);
    Eigen::Vector3d parse_sensor_3d_bias(const nlohmann::json& sensor_json);

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

    ParsedStepOptions parse_step_options(const nlohmann::json& frame_json);
    vehicles::NEDFrameECEFStepOptions parse_NEDFrameECEF_step_options(const nlohmann::json& frame_json);
    vehicles::FRDFrameECEFStepOptions parse_FRDFrameECEF_step_options(const nlohmann::json& frame_json);
    vehicles::FRDFrameNEDStepOptions parse_FRDFrameNED_step_options(const nlohmann::json& frame_json);
    vehicles::STABFrameFRDStepOptions parse_STABFrameFRD_step_options(const nlohmann::json& frame_json);
    vehicles::WINDFrameSTABStepOptions parse_WINDFrameSTAB_step_options(const nlohmann::json& frame_json);

    aerodynamics::DynamicDerivatives parse_dynamic_derivatives(const nlohmann::json& dyn_json);
    aerodynamics::ControlDerivatives parse_control_derivatives(const nlohmann::json& ctrl_json);
    control::ControlSurfaceLimits parse_control_surface_limits(const nlohmann::json& limits_json);
    control::ControlProperties parse_control_properties(const nlohmann::json& control_json);

    nlohmann::json read_json_file(const std::filesystem::path& path);
    std::filesystem::path resolve_config_path(const std::filesystem::path& run_path, const std::string& config_path);
    std::filesystem::path resolve_run_config_entry_path(const std::string& key);

    void _validate_initialization_config(const nlohmann::json& cfg, bool trim_bool);
    void _validate_NEDFrameECEF_initialization_config(const nlohmann::json& frame_json);
    void _validate_FRDFrameECEF_initialization_config(const nlohmann::json& frame_json);
    void _validate_FRDFrameNED_initialization_config(const nlohmann::json& frame_json);
    void _validate_WINDFrameSTAB_initialization_config(const nlohmann::json& frame_json);

    void create_dir(const std::string& dir);
    void write_txt(const std::string& text, const std::string& dir, const std::string& fname);
    void write_json(const nlohmann::json& cfg, const std::string& dir, const std::string& fname);
    void dump_configs(const std::string& dir);
    void save_vector_to_file(std::vector<int>& data, std::string fname);

    aerodynamics::AerodynamicProperties parse_aerodynamics_config();
    control::ControlProperties parse_control_config();
    vehicles::StepOptions parse_initialization_config(bool trim_enabled);
    structural::StructuralProperties parse_structural_config();
    avionics::AvionicsProperties parse_avionics_config();

    struct DataMatrix {
        Eigen::MatrixXd data;
        int n_rows;
        int n_cols;

        DataMatrix(const Eigen::MatrixXd& d);
        void write_csv(const std::string& dir, const std::string& fname) const;
        void insert(int t, const Eigen::VectorXd data, double dt);

    };

    std::string get_datetime();
}
