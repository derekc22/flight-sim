#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "simulation/transforms/transforms.hpp"


int main() {
    

    // rot
    Eigen::Matrix3d R01 = transforms::eul2R_extr(global::deg2rad(90), global::deg2rad(30), global::deg2rad(45), "ZYX");
    Eigen::Vector3d v0(1.0, 0.0, 0.0);
    Eigen::Vector3d v1 = R01 * v0;
    std::cout << v1 << std::endl;
    std::cout << std::endl;

    std::array<double, 3> eul01 = transforms::R2eul_extr(R01, "ZYX");
    for (const auto& i : eul01){
            std::cout << global::rad2deg(i) << ", ";
        }

    std::cout << std::endl;
    std::cout << std::endl;


    Eigen::Matrix3d CAB = transforms::eul2C_extr(global::deg2rad(90), global::deg2rad(30), global::deg2rad(45), "ZYX");
    Eigen::Vector3d vA(1.0, 0.0, 0.0);
    Eigen::Vector3d vB = CAB * vA;
    std::cout << vB << std::endl;
    std::cout << std::endl;

    std::array<double, 3> eulAB = transforms::C2eul_extr(CAB, "ZYX");
    for (const auto& i : eulAB){
            std::cout << global::rad2deg(i) << ", ";
        }

    std::cout << std::endl;
    std::cout << std::endl;


    Eigen::Vector3d vB1 = R01 * vB;
    std::cout << vB1 << std::endl;
    std::cout << std::endl;

    std::cout << transforms::RfromC(CAB) << std::endl;
    std::cout << std::endl;
    std::cout << transforms::CfromR(R01) << std::endl;
    std::cout << std::endl;
    std::cout << CAB << std::endl;










    // quat
    Eigen::Quaterniond q01 = transforms::eul2quatR_extr(global::deg2rad(90), global::deg2rad(30), global::deg2rad(45), "ZYX");
    Eigen::Vector3d v0(1.0, 0.0, 0.0);
    Eigen::Vector3d v1 = q01 * v0;
    std::cout << v1 << std::endl;
    std::cout << std::endl;

    std::array<double, 3> eul01 = transforms::quatR2eul_extr(q01, "ZYX");
    for (const auto& i : eul01){
            std::cout << global::rad2deg(i) << ", ";
        }

    std::cout << std::endl;
    std::cout << std::endl;


    Eigen::Quaterniond qAB = transforms::eul2quatC_extr(global::deg2rad(90), global::deg2rad(30), global::deg2rad(45), "ZYX");
    Eigen::Vector3d vA(1.0, 0.0, 0.0);
    Eigen::Vector3d vB = qAB * vA;
    std::cout << vB << std::endl;
    std::cout << std::endl;

    std::array<double, 3> eulAB = transforms::quatC2eul_extr(qAB, "ZYX");
    for (const auto& i : eulAB){
            std::cout << global::rad2deg(i) << ", ";
        }

    std::cout << std::endl;
    std::cout << std::endl;


    Eigen::Vector3d vB1 = q01 * vB;
    std::cout << vB1 << std::endl;
    std::cout << std::endl;

    std::cout << transforms::RfromC(transforms::quat2rot(qAB)) << std::endl;
    std::cout << std::endl;
    std::cout << transforms::CfromR(transforms::quat2rot(q01)) << std::endl;
    std::cout << std::endl;
    std::cout << transforms::quat2rot(qAB) << std::endl;
}