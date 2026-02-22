// #pragma once
// #include <Eigen/Dense>
// #include <unsupported/Eigen/CXX11/Tensor>

// namespace messages {

// class State {
//     public:
//     Eigen::Vector3d pos;
//     Eigen::Vector3d vel;
//     Eigen::Quaterniond att; 
//     Eigen::Vector3d omg;

//     State(Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Quaterniond& att, Eigen::Vector3d& omg);
// };

// class Frame {
//     public:
//     Eigen::Vector3d i;
//     Eigen::Vector3d j;
//     Eigen::Vector3d k; 
//     Eigen::Vector3d origin;

//     Frame(Eigen::Vector3d& i, Eigen::Vector3d& j, Eigen::Vector3d& k, Eigen::Vector3d& j, Eigen::Vector3d& origin);
// };

// class Image {
//     public:
//     Eigen::Tensor<double, 3> img(64, 64, 3);

//     Image(Eigen::Tensor<double, 3>& img);
// }

// class ControlCommand {
//     public:

// }

// }