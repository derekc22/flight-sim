// #pragma once

// #include <Eigen/Dense>
// #include <cmath>
// #include <tuple>
// #include <algorithm>
// #include "simulation/estimation/kalman.hpp"
// #include "simulation/dynamics/dynamics.hpp"
// #include "simulation/trim/trim.hpp"

// namespace estimation {

//     struct EstimationOutput {
//         dynamics::RigidBodyState state_estimate;
//     };

//     struct KalmanFilterInput {
//         const linearization::TrimStateJacobian& A;
//         const linearization::TrimInputJacobian& B;
//         const Eigen::MatrixXd& C;
//         const Eigen::MatrixXd& P0;
//         const Eigen::MatrixXd& Q0; 
//         const Eigen::MatrixXd& R0;
//     };

//     enum EstimatorType {
//         None,
//         // Kalman Filter
//         LinearKalmanFilter,
//         ExtendedKalmanFilter,
//     };

//     struct EstimationProperties {
//         KalmanFilter kalman_filter;

//         EstimationOutput step(
//             const dynamics::RigidBodyState& zN_t, 
//             const linearization::TrimLinearization& lin_sol, 
//             const trim::TrimSolution& trim_sol, 
//             const control::SurfaceActuatorInputs& u_surface_actual_prev, 
//             const control::PropulsorActuatorInputs& u_propulsor_actual_prev
//         );

//     };

// }
