#pragma once
#include <Eigen/Dense>
#include "simulation/actuators/public/data/types.hpp"
#include "simulation/autodiff/public/data/types.hpp"
#include "simulation/control/public/data/types.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/linearization/public/data/types.hpp"
#include "simulation/operating/public/data/types.hpp"
#include "simulation/trim/public/data/types.hpp"

namespace estimation {

    enum class EstimatorType {
        None,
        LinearKalmanFilter,
        ExtendedKalmanFilter
    };

    struct LinearKalmanEstimatorInput {
        dynamics::RigidBodyState Yt;
        operating::OperatingPoint_T<double> operating_point;
        linearization::LocalLinearization lin_sol;
        actuators::ActuatorInputs_T<double> u_actual_t_1;
    };

    struct ExtendedKalmanEstimatorInput {
        dynamics::RigidBodyState Yt;
        actuators::ActuatorInputs_T<double> u_actual_t_1;
        autodiff::AutoDiffModel& model;
        operating::OperatingConditions conditions;
    };

    struct KalmanState {
        dynamics::StateVector_T<double> zt;  // state estimate
        Eigen::MatrixXd Pt;  // state estimate error covariance matrix
    };

    struct KalmanFilterParameters {
        Eigen::MatrixXd P0; // initial state estimate error covariance matrix
        Eigen::MatrixXd Q;  // measurement noise covariance matrix
        Eigen::MatrixXd R;  // process noise covariance matrix
    };

    struct LinearKalmanFilterParameters : KalmanFilterParameters {};

    struct ExtendedKalmanFilterParameters : KalmanFilterParameters {};

    struct EstimationManagerInput {
        const dynamics::RigidBodyState& Yt;
        const trim::TrimSolution& trim_sol;
        const linearization::LocalLinearization& lin_sol;
        autodiff::AutoDiffModel& model;
        const control::ControlOutput& u_actual_t_1;
        const operating::OperatingConditions& conditions;
        double dt;
    };

    struct EstimationManagerOutput {
        dynamics::RigidBodyState Zt;  // state estimate
    };

}
