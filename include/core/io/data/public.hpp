#pragma once
#include <optional>
#include <string>
#include <Eigen/Dense>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"

namespace io {

    struct DataTable {
        Eigen::MatrixXd data;
        int n_rows;
        int n_cols;

        DataTable(const Eigen::MatrixXd& d);
        void insert(int t, const Eigen::VectorXd& data);
    };

    struct DataContext {
        const dynamics::RigidBodyState& Xt;
        const dynamics::RigidBodyState& Yt;
        const dynamics::RigidBodyState& Zt;
        const actuators::SurfaceActuatorInputs_T<double>& u_surface;
        const actuators::PropulsorActuatorInputs_T<double>& u_propulsor;
        const dynamics::Wrench& WB_net;
        const dynamics::Wrench& WB_aero;
        const dynamics::Wrench& WB_propulsive;
        const guidance::GuidanceSetpoint& setpoint;
        const atmospheric::Wind& windB;
    };

    struct DataManager {
        bool data_bool;
        bool control_bool;
        bool avionics_bool;
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
        void save(const std::string& data_dir_path);
        DataManager(int tf, bool data_bool, bool control_bool, bool avionics_bool, bool estimation_bool, bool wind_bool);

    };
}
