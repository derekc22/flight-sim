#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "simulation/transforms/transforms.hpp"

int main() {

    // a
    double a = global::deg2rad(90);

    Eigen::Quaterniond q_xy = transforms::qx(a) * transforms::qy(a);
    Eigen::Quaterniond q_yx = transforms::qy(a) * transforms::qx(a);

    Eigen::Vector3d v(0,0,1);

    Eigen::Vector3d v_xy = transforms::active_quat(q_xy, v);
    Eigen::Vector3d v_yx = transforms::active_quat(q_yx, v);

    std::cout << "v_xy = " << v_xy.transpose() << "\n";
    std::cout << "v_yx = " << v_yx.transpose() << "\n";
    // Expected: v_xy ~ (1,0,0), v_yx ~ (0,-1,0)



    // b
    double psi   = global::deg2rad(90);
    double theta = 0.0;
    double phi   = 0.0;

    Eigen::Quaterniond q = transforms::eul2quatR_extr(psi, theta, phi, "ZYX");
    std::cout << q.w() << ", " << q.x() << ", " << q.y() << ", " << q.z() << "\n";
    // Expected: ~ 0.70710678, 0, 0, 0.70710678



    // c
    double psi   = global::deg2rad(30);
    double theta = global::deg2rad(-20);
    double phi   = global::deg2rad(10);

    Eigen::Quaterniond q = transforms::eul2quatR_extr(psi, theta, phi, "ZYX");
    Eigen::Vector3d eul  = transforms::quatR2eul_extr(q, "ZYX");

    std::cout << global::rad2deg(eul[0]) << ", "
            << global::rad2deg(eul[1]) << ", "
            << global::rad2deg(eul[2]) << "\n";
    // Expected: ~ 30, -20, 10



    // d
    Eigen::Quaterniond q21 = transforms::qx(global::deg2rad(180));
    Eigen::Vector3d v1(0,1,2);

    Eigen::Vector3d v2 = transforms::passive_quat(q21, v1);
    std::cout << v2.transpose() << "\n";
    // Expected: (0, -1, -2)




}