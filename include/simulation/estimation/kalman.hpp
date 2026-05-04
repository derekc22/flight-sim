// #pragma once

// #include <Eigen/Dense>

// namespace estimation {

//     struct KalmanState {
//         Eigen::VectorXd x;
//         Eigen::MatrixXd P;
//     };

//     template <typename EstimatorOutput>
//     using KalmanFilter = std::function<RigidBodyStateEstimate(const EstimatorInput&)>;


//     StateEstimate EstimationProperties::step(
//         const dynamics::RigidBodyState& zN_t, 
//         const actuators::SurfaceActuators& surface_actuators
//     );
   
// }
