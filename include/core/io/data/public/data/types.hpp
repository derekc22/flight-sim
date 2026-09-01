#pragma once
#include <Eigen/Dense>
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/guidance/public/data/types.hpp"

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

}
