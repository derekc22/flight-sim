#include <Eigen/Dense>
#include <iostream>
#include "simulation/transforms/transforms.hpp"

int main() {

    double phi = global::deg2rad(45);    // roll (x)
    double theta = global::deg2rad(-45); // pitch (y)
    double psi = global::deg2rad(-90);   // yaw (z)



    // a
    Eigen::Matrix3d R_zyx = transforms::eul2R_intr(psi, theta, phi, "ZYX");
    std::cout << R_zyx << std::endl;
    std::cout << std::endl;



    // b
    Eigen::Vector3d eul_zyx = transforms::R2eul_intr(R_zyx, "ZYX");
    for (const auto& i : eul_zyx){
        std::cout << global::rad2deg(i) << ", ";
    }
    std::cout << std::endl;
    std::cout << std::endl;



    // c
    /**
        @deprecated
        The commented version is correct but deprecated. See new version below
     */
    // Eigen::Matrix3d R_NB = transforms::eul2R_intr(global::deg2rad(90), 0, 0, "ZYX");
    // Eigen::Vector3d v_N(1.0, 0.0, 0.0);
    // Eigen::Vector3d v_B = transforms::passive_R(R_NB, v_N);
    // for (const auto& i : v_B){
    //     std::cout << i << ", ";
    // }
    // std::cout << std::endl;
    // std::cout << std::endl;
    Eigen::Matrix3d C_NB = transforms::eul2C_intr(global::deg2rad(90), 0, 0, "ZYX");
    Eigen::Vector3d vN(1.0, 0.0, 0.0);
    Eigen::Vector3d vB = C_NB * vN;
    for (const auto& i : vB){
        std::cout << i << ", ";
    }
    std::cout << std::endl;
    std::cout << std::endl;


    // d
    /**
        @deprecated
        This commented version is incorrect if meant to apply a coordinate transformation. See new version below
     */
    // Eigen::Vector3d v(1.0, 0.0, 0.0);

    // Eigen::Matrix3d R = transforms::eul2R_intr(global::deg2rad(90), 0, 0, "ZYX");
    // Eigen::Vector3d t(2.0, 0.0, 0.0);

    // Eigen::Affine3d Rt = transforms::makeH(R, t);
    // Eigen::Vector3d u = transforms::applyH(Rt, v);
    // for (const auto& i : u){
    //     std::cout << i << ", ";
    // }
    // std::cout << std::endl;
    // std::cout << std::endl;

    // Eigen::Affine3d Rtinv = transforms::makeHinv(R, t);
    // Eigen::Vector3d w = transforms::applyInverseAffine(Rt, u);
    // for (const auto& i : w){
    //     std::cout << i << ", ";
    // }
    // std::cout << std::endl;
    // std::cout << std::endl;

    Eigen::Vector3d v(1.0, 0.0, 0.0);

    Eigen::Matrix3d C = transforms::eul2C_intr(global::deg2rad(90), 0, 0, "ZYX");
    Eigen::Vector3d d(2.0, 0.0, 0.0);

    Eigen::Matrix4d H = transforms::makeHC(C, d, "rotate");
    Eigen::Vector3d u = transforms::apply_hom(H, v);
    for (const auto& i : u){
        std::cout << i << ", ";
    }
    std::cout << std::endl;
    std::cout << std::endl;

    Eigen::Matrix4d Hinv = transforms::make_Hinv(H);
    Eigen::Vector3d w = transforms::apply_hom(Hinv, u);
    for (const auto& i : w){
        std::cout << i << ", ";
    }
    std::cout << std::endl;
    std::cout << std::endl;
}
