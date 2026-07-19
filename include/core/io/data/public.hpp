#pragma once
#include <optional>
#include <string>
#include <Eigen/Dense>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/runner/public.hpp"

namespace io {

    struct DataMatrix {
        int n_rows;
        int n_cols;
        Eigen::MatrixXd data;

        DataMatrix(int n_rows, int n_cols);
        void insert(int t, const Eigen::VectorXd& input);
    };

    struct DataContext {
        dynamics::RigidBodyState Xt;
        dynamics::RigidBodyState Yt;
        dynamics::RigidBodyState Zt;
        actuators::SurfaceActuatorInputs_T<double> u_surface;
        actuators::PropulsorActuatorInputs_T<double> u_propulsor;
        actuators::SurfaceActuatorInputs_T<double> u_surface_commanded;
        actuators::PropulsorActuatorInputs_T<double> u_propulsor_commanded;
        dynamics::Wrench WB_net;
        dynamics::Wrench WB_aerodynamic;
        dynamics::Wrench WB_propulsive;
        guidance::GuidanceSetpoint setpoint;
        atmospheric::Wind windB;
    };

    struct DataManager {
        runner::JSONFlags json_flags;

        std::optional<DataMatrix> p_DM;
        std::optional<DataMatrix> p_measured_DM;
        std::optional<DataMatrix> p_estimated_DM;
        std::optional<DataMatrix> eul_DM;
        std::optional<DataMatrix> eul_measured_DM;
        std::optional<DataMatrix> eul_estimated_DM;
        std::optional<DataMatrix> eul_setpoint_DM;
        std::optional<DataMatrix> w_DM;
        std::optional<DataMatrix> w_measured_DM;
        std::optional<DataMatrix> w_estimated_DM;
        std::optional<DataMatrix> w_setpoint_DM;
        std::optional<DataMatrix> v_DM;
        std::optional<DataMatrix> v_measured_DM;
        std::optional<DataMatrix> v_estimated_DM;
        std::optional<DataMatrix> v_setpoint_DM;
        std::optional<DataMatrix> u_surface_DM;
        std::optional<DataMatrix> u_propulsor_DM;
        std::optional<DataMatrix> u_surface_commanded_DM;
        std::optional<DataMatrix> u_propulsor_commanded_DM;
        std::optional<DataMatrix> F_net_DM;
        std::optional<DataMatrix> M_net_DM;
        std::optional<DataMatrix> F_aerodynamic_DM;
        std::optional<DataMatrix> M_aerodynamic_DM;
        std::optional<DataMatrix> F_propulsive_DM;
        std::optional<DataMatrix> M_propulsive_DM;
        std::optional<DataMatrix> windB_DM;

        void step(int t, const DataContext& context);
        void save(const std::string& data_dir_path);
        DataManager(int tf, const runner::JSONFlags& json_flags);

    };
}
