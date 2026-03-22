#include <iomanip>
#include <filesystem>
#include <fstream>
#include <vector>
#include <string>
#include <stdexcept>
#include <optional>
#include <Eigen/Dense>
#include "core/io/io.hpp"

namespace io {

    Eigen::Vector3d parse_vector3d(const nlohmann::json& values) {
        if (!values.is_array() || values.size() != 3) {
            throw std::runtime_error("expected a 3-element array");
        }

        return Eigen::Vector3d(
            values.at(0).get<double>(),
            values.at(1).get<double>(),
            values.at(2).get<double>()
        );
    }

    Eigen::Vector4d parse_vector4d(const nlohmann::json& values) {
        if (!values.is_array() || values.size() != 4) {
            throw std::runtime_error("expected a 4-element array");
        }

        return Eigen::Vector4d(
            values.at(0).get<double>(),
            values.at(1).get<double>(),
            values.at(2).get<double>(),
            values.at(3).get<double>()
        );
    }

    Eigen::Matrix3d parse_matrix3d(const nlohmann::json& values) {
        if (!values.is_array() || values.size() != 3) {
            throw std::runtime_error("expected a 3x3 array");
        }

        Eigen::Matrix3d out;
        for (int row = 0; row < 3; ++row) {
            const auto& row_values = values.at(row);
            if (!row_values.is_array() || row_values.size() != 3) {
                throw std::runtime_error("expected a 3x3 array");
            }

            for (int col = 0; col < 3; ++col) {
                out(row, col) = row_values.at(col).get<double>();
            }
        }

        return out;
    }

    Eigen::Matrix4d parse_matrix4d(const nlohmann::json& values) {
        if (!values.is_array() || values.size() != 4) {
            throw std::runtime_error("expected a 4x4 array");
        }

        Eigen::Matrix4d out;
        for (int row = 0; row < 4; ++row) {
            const auto& row_values = values.at(row);
            if (!row_values.is_array() || row_values.size() != 4) {
                throw std::runtime_error("expected a 4x4 array");
            }

            for (int col = 0; col < 4; ++col) {
                out(row, col) = row_values.at(col).get<double>();
            }
        }

        return out;
    }

    Eigen::Quaterniond parse_quaterniond(const nlohmann::json& values) {
        const Eigen::Vector4d q = parse_vector4d(values);
        return Eigen::Quaterniond(q(0), q(1), q(2), q(3));
    }

    ParsedStepOptions parse_step_options(const nlohmann::json& frame_json) {
        ParsedStepOptions fields;

        if (frame_json.contains("H")) {
            fields.H = dynamics::HomogenousFrameTransformationMatrix{ parse_matrix4d(frame_json.at("H")) };
        }
        if (frame_json.contains("C")) {
            fields.C = dynamics::OrientationMatrix{ parse_matrix3d(frame_json.at("C")) };
        }
        if (frame_json.contains("p")) {
            fields.p = dynamics::Position{ parse_vector3d(frame_json.at("p")) };
        }
        if (frame_json.contains("q")) {
            fields.q = dynamics::OrientationQuaternion{ parse_quaterniond(frame_json.at("q")) };
        }
        if (frame_json.contains("eul")) {
            fields.eul = dynamics::EulerAngles{ parse_vector3d(frame_json.at("eul")) };
        }
        if (frame_json.contains("C_dot")) {
            fields.C_dot = dynamics::OrientationMatrixRate{ parse_matrix3d(frame_json.at("C_dot")) };
        }
        if (frame_json.contains("q_dot")) {
            fields.q_dot = dynamics::OrientationQuaternionRate{ parse_quaterniond(frame_json.at("q_dot")) };
        }
        if (frame_json.contains("w")) {
            fields.w = dynamics::AngularVelocity{ parse_vector3d(frame_json.at("w")) };
        }
        if (frame_json.contains("eul_dot")) {
            fields.eul_dot = dynamics::EulerAngleRates{ parse_vector3d(frame_json.at("eul_dot")) };
        }
        if (frame_json.contains("wq")) {
            fields.wq = dynamics::AngularVelocityQuaternion{ parse_quaterniond(frame_json.at("wq")) };
        }
        if (frame_json.contains("v")) {
            fields.v = dynamics::LinearVelocity{ parse_vector3d(frame_json.at("v")) };
        }
        if (frame_json.contains("lat")) {
            fields.lat = geography::Latitude{ global::deg_to_rad(frame_json.at("lat").get<double>()) };
        }
        if (frame_json.contains("lon")) {
            fields.lon = geography::Longitude{ global::deg_to_rad(frame_json.at("lon").get<double>()) };
        }
        if (frame_json.contains("alt")) {
            fields.alt = geography::Altitude{ frame_json.at("alt").get<double>() };
        }
        if (frame_json.contains("alpha")) {
            fields.alpha = aerodynamics::AngleOfAttack{ frame_json.at("alpha").get<double>() };
        }
        if (frame_json.contains("beta")) {
            fields.beta = aerodynamics::SideslipAngle{ frame_json.at("beta").get<double>() };
        }

        return fields;
    }

    vehicles::NEDFrameECEFStepOptions parse_NEDFrameECEF_step_options(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        return vehicles::NEDFrameECEFStepOptions{
            .lat_NE = fields.lat,
            .lon_NE = fields.lon,
            .alt_NE = fields.alt,
        };
    }

    vehicles::FRDFrameECEFStepOptions parse_FRDFrameECEF_step_options(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        return vehicles::FRDFrameECEFStepOptions{
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
        return vehicles::FRDFrameNEDStepOptions{
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
        return vehicles::STABFrameFRDStepOptions{
            .alpha = fields.alpha,
        };
    }

    vehicles::WINDFrameSTABStepOptions parse_WINDFrameSTAB_step_options(const nlohmann::json& frame_json) {
        const ParsedStepOptions fields = parse_step_options(frame_json);
        return vehicles::WINDFrameSTABStepOptions{
            .beta = fields.beta,
        };
    }

    aerodynamics::DynamicDerivatives parse_dynamic_derivatives(const nlohmann::json& dyn_json) {
        return aerodynamics::DynamicDerivatives{
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
        return aerodynamics::ControlDerivatives{
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

    control::ControlSurfaceLimits parse_control_surface_limits(const nlohmann::json& limits_json) {
        return control::ControlSurfaceLimits{
            .elevator_max = global::deg_to_rad(limits_json.value("elevator_max", 0.0)),
            .aileron_max = global::deg_to_rad(limits_json.value("aileron_max", 0.0)),
            .rudder_max = global::deg_to_rad(limits_json.value("rudder_max", 0.0)),
            .flap_max = global::deg_to_rad(limits_json.value("flap_max", 0.0)),
            .spoiler_max = global::deg_to_rad(limits_json.value("spoiler_max", 0.0)),
        };
    }

    control::ControlProperties parse_control_properties(const nlohmann::json& control_json) {
        control::ControlProperties control_properties{};

        if (control_json.contains("control_surface_limits")) {
            control_properties.limits = parse_control_surface_limits(control_json.at("control_surface_limits"));
        }

        return control_properties;
    }

    nlohmann::json read_json_file(const std::filesystem::path& path) {
        std::ifstream file(path);
        if (!file.is_open()) {
            throw std::runtime_error("failed to open file: " + path.string());
        }

        nlohmann::json cfg;
        file >> cfg;
        return cfg;
    }

    std::filesystem::path resolve_config_path(
        const std::filesystem::path& run_path,
        const std::string& config_path
    ) {
        const std::filesystem::path path{ config_path };
        if (path.is_absolute()) {
            return path;
        }

        return run_path.parent_path() / path;
    }

    std::filesystem::path resolve_run_config_entry_path(const std::string& key) {
        const auto run_path = std::filesystem::path("config") / "run.json";
        const auto run_config = read_json_file(run_path);
        return resolve_config_path(run_path, run_config.at(key).get<std::string>());
    }


    void create_dir(const std::string& dir) {
        std::filesystem::create_directories(dir);
    }

    void save_vector_to_file(std::vector<int>& data, std::string fname){
        std::string path_name = "data/" + fname + ".csv";
        std::ofstream file_v(path_name);
        for (size_t i = 0; i < data.size(); i++){
            file_v << data[i];
            if (i < data.size() - 1) file_v << ",";  // comma delimiter
        }
        file_v.close();
    }

    aerodynamics::AerodynamicProperties parse_aerodynamics_config() {
        const auto aerodynamics_path = resolve_run_config_entry_path("aerodynamics_config_path");
        const auto cfg = read_json_file(aerodynamics_path);

        const auto& surfaces_json = cfg.at("surfaces");
        if (!surfaces_json.is_array()) {
            throw std::runtime_error("io::parse_aerodynamics_config expected 'surfaces' to be an array");
        }

        std::vector<aerodynamics::Surface> surfaces;
        surfaces.reserve(surfaces_json.size());

        for (const auto& surface_json : surfaces_json) {
            surfaces.push_back(aerodynamics::Surface{
                .id = surface_json.at("id").get<std::string>(),
                .chord = surface_json.at("chord").get<double>(),
                .span = surface_json.at("span").get<double>(),
                .p_ref = parse_vector3d(surface_json.at("p_ref")),
                .n = parse_vector3d(surface_json.at("n")),
                .CL0 = surface_json.at("CL0").get<double>(),
                .e = surface_json.at("e").get<double>(),
                .i = surface_json.at("i").get<double>(),
                .CD0 = surface_json.at("CD0").get<double>(),
                .CDa = surface_json.at("CDa").get<double>(),
                .a0 = surface_json.at("a0").get<double>(),
                .CM0 = surface_json.at("CM0").get<double>(),
                .CMa = surface_json.at("CMa").get<double>(),
                .dyn = surface_json.contains("dyn")
                    ? parse_dynamic_derivatives(surface_json.at("dyn"))
                    : aerodynamics::DynamicDerivatives{},
                .ctrl = surface_json.contains("ctrl")
                    ? parse_control_derivatives(surface_json.at("ctrl"))
                    : aerodynamics::ControlDerivatives{},
            });
        }

        return aerodynamics::AerodynamicProperties{ surfaces };
    }

    control::ControlProperties parse_control_config() {
        const auto control_path = resolve_run_config_entry_path("control_config_path");
        const auto cfg = read_json_file(control_path);
        return parse_control_properties(cfg);
    }

    vehicles::StepOptions parse_init_options_config() {
        const auto init_path = resolve_run_config_entry_path("initialization_config_path");
        const auto cfg = read_json_file(init_path);

        vehicles::StepOptions opts;

        if (cfg.contains("NEDFrameECEF")) {
            opts.NEDFrameECEFStepOpts = parse_NEDFrameECEF_step_options(cfg.at("NEDFrameECEF"));
        }

        if (cfg.contains("FRDFrameECEF")) {
            opts.FRDFrameECEFStepOpts = parse_FRDFrameECEF_step_options(cfg.at("FRDFrameECEF"));
        }

        if (cfg.contains("FRDFrameNED")) {
            opts.FRDFrameNEDStepOpts = parse_FRDFrameNED_step_options(cfg.at("FRDFrameNED"));
        }

        if (cfg.contains("STABFrameFRD")) {
            opts.STABFrameFRDStepOpts = parse_STABFrameFRD_step_options(cfg.at("STABFrameFRD"));
        }

        if (cfg.contains("WINDFrameSTAB")) {
            opts.WINDFrameSTABStepOpts = parse_WINDFrameSTAB_step_options(cfg.at("WINDFrameSTAB"));
        }

        return opts;
    }

    structural::StructuralProperties parse_structural_config() {
        const auto structural_path = resolve_run_config_entry_path("structural_config_path");
        const auto cfg = read_json_file(structural_path);

        const auto& geometries_json = cfg.at("geometries");
        if (!geometries_json.is_array()) {
            throw std::runtime_error("io::parse_structural_config expected 'geometries' to be an array");
        }

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

        return structural::StructuralProperties{ geometries };
    }

    DataMatrix::DataMatrix(const Eigen::MatrixXd& d) : data(d), n_rows(static_cast<int>(data.rows())), n_cols(static_cast<int>(data.cols())) {};

    void DataMatrix::write_csv(const std::string& dir, const std::string& fname) const {

        create_dir(dir);

        // std::string path_name = dir + "/" + fname + ".csv";
        auto path_name = std::filesystem::path(dir) / (fname + ".csv");

        std::ofstream file_m(path_name);
        if (!file_m.is_open()) {
            throw std::runtime_error("Failed to open file: " + path_name.string());
        }

        for (int i = 0; i < n_rows; ++i) {
            for (int j = 0; j < n_cols; ++j) {
                file_m << data(i, j);
                if (j < n_cols - 1) file_m << ","; // comma delimiter
            }
            file_m << "\n";
        }
        file_m.close();
    }

    void DataMatrix::set(int t, const Eigen::VectorXd input, double dt){
        if (input.cols() > 1) { throw std::runtime_error("io::DataMatrix::set Eigen::Matrix passed for 'input', expected Eigen::Vector"); }
        if (input.rows() > n_cols-1) { throw std::runtime_error("io::DataMatrix::set Number of rows in 'input' exceeds number of columns in DataMatrix"); }
        if (t > n_rows-1) { throw std::runtime_error("io::DataMatrix::set Input index 't' exceeds number of rows in DataMatrix"); }

        data(t, 0) = t * dt;
        Eigen::Index cols_to_copy = data.cols() - 1;
        data.block(t, 1, 1, cols_to_copy) = input.transpose(); // startRow, startCol, blockRows, blockCols.
    }



}
