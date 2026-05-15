#pragma once
#include <optional>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/atmospheric/public.hpp"

namespace io {

    void create_dir(const std::string& dir);
    void write_txt(const std::string& text, const std::string& dir, const std::string& fname);
    void write_csv(const Eigen::MatrixXd& data, const std::string& dir, const std::string& fname);
    void save_vector_to_file(const std::vector<int>& data, const std::string& fname);

    struct DataTable {
        Eigen::MatrixXd data;
        int n_rows;
        int n_cols;

        DataTable(const Eigen::MatrixXd& d);
        void insert(int t, const Eigen::VectorXd& data);
    };

    struct DataContext {
        const dynamics::RigidBodyState& xN_t;
        const dynamics::RigidBodyState& yN_t;
        const dynamics::RigidBodyState& zN_t;
        const actuators::SurfaceActuatorInputs_T<double>& u_surface_actual;
        const actuators::PropulsorActuatorInputs_T<double>& u_propulsor_actual;
        const dynamics::Wrench& WB_net;
        const dynamics::Wrench& WB_aero;
        const dynamics::Wrench& WB_propulsive;
        const guidance::GuidanceSetpoint& setpoint;
        const atmospheric::Wind& windB;
    };

    struct DataManager {
        bool data_bool;
        bool control_bool;
        bool sensor_bool;
        bool estimation_bool;
        bool wind_bool;

        std::optional<DataTable> p_DT;
        std::optional<DataTable> p_meas_DT;
        std::optional<DataTable> p_est_DT;
        std::optional<DataTable> eul_DT;
        std::optional<DataTable> eul_meas_DT;
        std::optional<DataTable> eul_est_DT;
        std::optional<DataTable> eul_setpoint_DT;
        std::optional<DataTable> w_DT;
        std::optional<DataTable> w_meas_DT;
        std::optional<DataTable> w_est_DT;
        std::optional<DataTable> w_setpoint_DT;
        std::optional<DataTable> v_DT;
        std::optional<DataTable> v_meas_DT;
        std::optional<DataTable> v_est_DT;
        std::optional<DataTable> v_setpoint_DT;
        std::optional<DataTable> u_surface_DT;
        std::optional<DataTable> u_propulsor_DT;
        std::optional<DataTable> F_net_DT;
        std::optional<DataTable> M_net_DT;
        std::optional<DataTable> F_aero_DT;
        std::optional<DataTable> M_aero_DT;
        std::optional<DataTable> F_prop_DT;
        std::optional<DataTable> M_prop_DT;
        std::optional<DataTable> windB_DT;

        void step(int t, const DataContext& context);
        void save(const std::string& out_dir_path);
        DataManager(int tf, bool data_bool, bool control_bool, bool sensor_bool, bool estimation_bool, bool wind_bool);

    };

    std::string get_datetime();
}
