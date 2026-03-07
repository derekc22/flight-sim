#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "simulation/transforms/transforms.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/global/global.hpp"
#include "simulation/frames/frames.hpp"
#include "simulation/vehicles/vehicles.hpp"
#include "core/io/io.hpp"


void q2(bool remove_omega_cross_v, double init_vel, const std::string& trial) {
    int tf = 10000;                       // seconds

    io::DataMatrix q2DM{ Eigen::MatrixXd::Zero(tf, 4) };

    // Initial attitude and position in NED
    dynamics::OrientationMatrix CNB{ global::I3 };                  // C_{N->B}(0)
    dynamics::Position pN_BN{ Eigen::Vector3d::Zero() };            // p^N_{NB}(0)

    // Inputs
    dynamics::AngularVelocity wB_BN{ Eigen::Vector3d(0, 0, 1) };       // rad/s, expressed in B

    // Initial body velocity
    dynamics::LinearVelocity vB_EB0{ Eigen::Vector3d(init_vel, 0, 0) };   // m/s expressed in B

    // This is the inertial, ground-track velocity (due north)
    const dynamics::LinearVelocity vN0{ CNB.data.transpose() * vB_EB0.data }; // v^N(0)

    // Net acceleration in NED is zero for perfect counter-gravity in this nominal case
    const dynamics::LinearAcceleration aN{ Eigen::Vector3d::Zero() };

    // Working copy of body velocity
    dynamics::LinearVelocity vB_EB = vB_EB0;

    for (int t = 0; t < tf; ++t) {
        // 1) Attitude propagation: C_{N->B}(t+dt)
        CNB = dynamics::_rot_kin(CNB, wB_BN);

        // 2) Velocity expressed in NED
        dynamics::LinearVelocity vN_EB;

        if (!remove_omega_cross_v) {
            // WITH omega x v:
            // v^N stays constant, v^B must rotate with the body: v^B = C_{N->B} v^N
            vB_EB.data = CNB.data * vN0.data;
            vN_EB = vN0; // constant due north
        } else {
            // WITHOUT omega x v:
            // v^B stays constant in body, so v^N = C_{B->N} v^B rotates
            vN_EB.data = CNB.data.transpose() * vB_EB.data;
        }

        // 3) Position propagation in NED
        pN_BN = dynamics::_trans_kin(pN_BN, vN_EB, aN);

        q2DM.data(t, 0) = t * dynamics::common::dt;
        q2DM.data.block<1, 3>(t, 1) = pN_BN.data.transpose();
    }

    q2DM.write_csv("data/hw3", "q2" + trial);
    std::cout << "Final pN_BN: " << pN_BN.data.transpose() << "\n";
}



int main() {
    
    // 1a
    Eigen::Vector3d pE_BE_t0(global::r_earth, 0, 0); // m
    Eigen::Vector3d vB_BE(150, 0, 0); // ms^-1

    double psi = global::deg2rad(0), theta = global::deg2rad(0), phi = global::deg2rad(0);
    Eigen::Matrix3d C_BN = transforms::eul2C_intr(psi, theta, phi, "ZYX");

    int tf = 1000; //s
    Eigen::Vector3d pE_BE_t;


    // 1a.i
    io::DataMatrix q1aiDM{ Eigen::MatrixXd::Zero(tf, 4) };

    double lat = global::deg2rad(0), lon = global::deg2rad(0);
    Eigen::Matrix3d C_NE = frames::CEN_from_lat_lon(lat, lon).transpose();
    Eigen::Matrix3d C_BE = transforms::chain_rot_extr(std::vector<Eigen::Matrix3d>{C_BN, C_NE});
    Eigen::Vector3d vE_BE = C_BE * vB_BE;

    // auto [pE_BE_tf, _] = dynamics::fwd_euler(pE_BE_t0, vE_BE, dynamics::common::f_cv, tf);
    // std::cout << pE_BE_tf << std::endl;
    // std::cout << std::endl;

    pE_BE_t = pE_BE_t0;
    for (int t = 0; t < tf; ++t){

        auto [pE_BE_t1, _] = dynamics::fwd_euler(pE_BE_t, vE_BE, dynamics::common::f_cv, 1);
        pE_BE_t = pE_BE_t1;

        q1aiDM.data(t, 0) = t * dynamics::common::dt;
        q1aiDM.data.block<1, 3>(t, 1) = pE_BE_t1.transpose();
    }

    q1aiDM.write_csv("data/hw3", "q1ai");
    std::cout << pE_BE_t << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;



    // 1a.ii
    io::DataMatrix q1aiiDM{ Eigen::MatrixXd::Zero(tf, 4) };

    double lat_t = global::deg2rad(0), lon_t = global::deg2rad(0);

    Eigen::Matrix3d C_NE_t;
    Eigen::Matrix3d C_BE_t;
    pE_BE_t = pE_BE_t0;
    Eigen::Vector3d vB_BE_t = vB_BE;
    Eigen::Vector3d vE_BE_t;

    for (int t = 0; t < tf; ++t){
        auto [lat_t, lon_t, alt] = frames::lat_lon_alt_from_pECEF(pE_BE_t);

        C_NE_t = frames::CEN_from_lat_lon(lat_t, lon_t).transpose();
        C_BE_t = transforms::chain_rot_extr(std::vector<Eigen::Matrix3d>{C_BN, C_NE_t});
        vE_BE_t = C_BE_t * vB_BE_t;

        auto [pE_BE_t1, _] = dynamics::fwd_euler(pE_BE_t, vE_BE_t, dynamics::common::f_cv, 1);

        pE_BE_t = pE_BE_t1;

        q1aiiDM.data(t, 0) = t * dynamics::common::dt;
        q1aiiDM.data.block<1, 3>(t, 1) = pE_BE_t1.transpose();
    }

    q1aiiDM.write_csv("data/hw3", "q1aii");
    std::cout << pE_BE_t << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;


    // 2a
    q2(true, 150, "a");
    q2(false, 150, "b");
    q2(false, 300, "c");


}