#pragma once
#include <Eigen/Dense>
#include <optional>
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

    struct EstimatorInputs {
        const dynamics::RigidBodyState Yt;
        std::optional<LinearKalmanEstimatorInput> linear_kalman_estimator_input;
        std::optional<ExtendedKalmanEstimatorInput> extended_kalman_estimator_input;
    };

    struct EstimationOutput {
        dynamics::RigidBodyState Zt;  // state estimate
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
        const dynamics::RigidBodyState& measured_state;
        const trim::TrimSolution& trim_solution;
        const linearization::LocalLinearization& linearization;
        autodiff::AutoDiffModel& model;
        const control::ControlOutput& previous_actual_inputs;
        const operating::OperatingConditions& conditions;
        double dt;
    };

    struct EstimationManagerOutput {
        dynamics::RigidBodyState estimated_state;
    };

}
