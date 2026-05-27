#pragma once
#include <Eigen/Dense>
#include <optional>
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/estimation/public.hpp"
#include "simulation/operating/public.hpp"

namespace vehicles { struct Aircraft; }
namespace estimation {

    struct KalmanState {
        dynamics::StateVector_T<double> z;  // state estimate
        Eigen::MatrixXd P;  // state estimate error covariance matrix
    };

    struct KalmanFilterInput {
        linearization::StateJacobian A; 
        linearization::InputJacobian B;
        linearization::OutputJacobian C;
        dynamics::StateVector_T<double> yN_t;
        actuators::ActuatorInputsVector_T<double> ut_1;
        operating::OperatingConditions conditions;
        vehicles::Aircraft& aircraft;
        bool nonlinear_bool;
    };

    struct KalmanFilterParameters {
        Eigen::MatrixXd P0; // initial state estimate error covariance matrix
        Eigen::MatrixXd Q;  // measurement noise covariance matrix
        Eigen::MatrixXd R;  // process noise covariance matrix
    };

    struct KalmanFilter {
        KalmanFilterParameters params;
        std::optional<KalmanState> state;

        KalmanFilter(const KalmanFilterParameters& params);
        KalmanState predict_linear(const KalmanFilterInput& filter_input);
        KalmanState correct_linear(const KalmanFilterInput& filter_input);
        KalmanState predict_nonlinear(const KalmanFilterInput& filter_input);
        KalmanState correct_nonlinear(const KalmanFilterInput& filter_input);

        KalmanState step(const KalmanFilterInput& filter_input);
    };
}
