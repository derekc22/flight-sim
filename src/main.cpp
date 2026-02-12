#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "simulation/transforms/transforms.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/global/global.hpp"
#include "simulation/frames/frames.hpp"
#include "simulation/vehicles/vehicles.hpp"
#include "core/io/io.hpp"


void q2(){

    // define frames
    frames::ECEFFrame ECEF;
    frames::NEDFrame NED;
    frames::FRDFrame BODY;

    double lat = 0.0, lon = 0.0;
    Eigen::Matrix3d _CEN = frames::CEN_from_lat_lon(lat, lon);
    Eigen::Vector3d _pE_NE(global::r_earth, 0, 0);
    NED.HEN.set(_CEN, _pE_NE);

    Eigen::Matrix3d _CNB = global::I3;
    Eigen::Vector3d _pN_BN(0, 0, 0);
    BODY.HNB.set(_CNB, _pN_BN);

    Eigen::Matrix3d _CEB = _CNB * _CEN;
    Eigen::Vector3d _pE_BE = _CEN.transpose() * _pN_BN + _pE_NE;
    BODY.HEB.set(_CEB, _pE_BE);
    
    // create vehicle
    double mass = 1; // kg
    BODY.wB_BN.data = Eigen::Vector3d(0, 0, 1); // rad/s
    vehicles::Aircraft plane{ .BODY = BODY, .mass = mass };








    int t = 0;
    int Tf = 100;

    Eigen::Vector3d vB_BE(150, 0, 0); // ms^-1

    for (t = 0; t < Tf; ++t){

        Eigen::Vector3d pE_EB = NED.HEN.p().data + BODY.HEN.C().data.transpose() * BODY.HNB.p().data;

        // gravity
        Eigen::Vector3d gE = frames::common::gECEF(pE_EB); // ms^-2, should be radial
        Eigen::Vector3d gB = BODY.HEB.C().data * gE;
        Eigen::Vector3d FgB = mass * gB;
        Eigen::Vector3d FcounterB = -FgB;
        Eigen::Vector3d a_netB = (FgB + FcounterB) / plane.mass; // should be 0 vector


        // Get new CEN, CEB, CBE from current lat/lon
        auto [lat, lon, alt] = frames::lat_lon_alt_from_xECEF(pE_BE);
        Eigen::Matrix3d CEN = frames::CEN_from_lat_lon(lat, lon); // E -> N
        Eigen::Matrix3d CEB = BODY.HNB.C().data * CEN;            // E -> B
        Eigen::Matrix3d CBE = CEB.transpose();                    // B -> E

        // orientation
        dynamics::OrientationMatrix CNB_t1 = dynamics::strap_down_kin(BODY.HNB.C(), BODY.wB_BN);

        // position
        dynamics::LinearVelocity pN_BN_dot { BODY.HNB.C().data.transpose() * BODY.vB_BN.data };
        dynamics::Position pN_BN_t1 = dynamics::trans_kin(BODY.HNB.p(), pN_BN_dot, BODY.aB_BN);

        // update
        BODY.HNB.set(CNB_t1.data, pN_BN_t1.data);
    }
}

// void q2()
// {
//     frames::NEDFrame NED;
//     frames::FRDFrame BODY;

//     // Initial NED at equator, prime meridian
//     const double lat0 = 0.0, lon0 = 0.0;
//     const Eigen::Matrix3d CEN0 = frames::CEN_from_lat_lon(lat0, lon0); // E -> N
//     const Eigen::Vector3d pE_NE0(global::r_earth, 0, 0);

//     NED.HEN.set(CEN0, pE_NE0);

//     // Initial body aligned with NED (FRD aligned with NED axes)
//     Eigen::Matrix3d CNB = global::I3;                 // N -> B
//     Eigen::Vector3d pN_BN = Eigen::Vector3d::Zero();  // position in N
//     BODY.HNB.set(CNB, pN_BN);

//     // Inputs
//     const double mass = 1.0;
//     const Eigen::Vector3d wB_BN(0, 0, 1);             // rad/s
//     Eigen::Vector3d vB_EB(150, 0, 0);                 // m/s, initial velocity in body

//     const double dt = dynamics::common::dt;
//     const double Tf = 100.0;
//     const int steps = static_cast<int>(Tf / dt);

//     for (int k = 0; k < steps; ++k)
//     {
//         // 1) Attitude update CNB
//         dynamics::OrientationMatrix CNB_t{CNB};
//         dynamics::BodyAxisRates w_t{wB_BN};
//         CNB = dynamics::strap_down_kin(CNB_t, w_t).data;

//         // 2) Gravity computed in E, expressed in B, then perfectly cancelled
//         const Eigen::Vector3d pE_BE = pE_NE0 + CEN0.transpose() * pN_BN; // p^E_EB = p^E_EN + C_NE p^N_NB
//         const Eigen::Vector3d gE = frames::common::gECEF(pE_BE);         // radial m/s^2

//         const Eigen::Matrix3d CEB = CNB * CEN0;                          // E -> B
//         const Eigen::Vector3d gB = CEB * gE;

//         const Eigen::Vector3d FgB = mass * gB;
//         const Eigen::Vector3d FcounterB = -FgB;
//         const Eigen::Vector3d aB = (FgB + FcounterB) / mass;             // = 0

//         // 3) Translational dynamics in body
//         const Eigen::Vector3d vB_dot = aB - wB_BN.cross(vB_EB);
//         vB_EB += vB_dot * dt;

//         // 4) Translational kinematics in NED
//         const Eigen::Vector3d vN_EB = CNB.transpose() * vB_EB;            // v^N = C_BN v^B
//         pN_BN += vN_EB * dt;

//         // 5) Store back into the frame object if you want to use it elsewhere
//         BODY.HNB.set(CNB, pN_BN);
//     }

//     std::cout << "Final pN_BN: " << pN_BN.transpose() << "\n";
// }

// void q2()
// {
//     frames::NEDFrame NED;
//     frames::FRDFrame BODY;

//     // Initial NED at equator, prime meridian
//     const double lat0 = 0.0, lon0 = 0.0;
//     const Eigen::Matrix3d CEN0 = frames::CEN_from_lat_lon(lat0, lon0); // E -> N
//     const Eigen::Vector3d pE_NE0(global::r_earth, 0, 0);

//     NED.HEN.set(CEN0, pE_NE0);

//     // Initial body aligned with NED (FRD aligned with NED axes)
//     Eigen::Matrix3d CNB = global::I3;                 // N -> B
//     Eigen::Vector3d pN_BN = Eigen::Vector3d::Zero();  // position in N
//     BODY.HNB.set(CNB, pN_BN);

//     // Inputs
//     const double mass = 1.0;
//     const Eigen::Vector3d wB_BN(0, 0, 1);             // rad/s
//     Eigen::Vector3d vB_EB(150, 0, 0);                 // m/s, initial velocity in body

//     const double dt = dynamics::common::dt;
//     const double Tf = 100.0;
//     const int steps = static_cast<int>(Tf / dt);

//     for (int k = 0; k < steps; ++k)
//     {
//         // 1) Attitude update CNB
//         dynamics::OrientationMatrix CNB_t{CNB};
//         dynamics::BodyAxisRates w_t{wB_BN};
//         CNB = dynamics::strap_down_kin(CNB_t, w_t).data;

//         // 2) Gravity computed in E, expressed in B, then perfectly cancelled
//         const Eigen::Vector3d pE_BE = pE_NE0 + CEN0.transpose() * pN_BN; // p^E_EB = p^E_EN + C_NE p^N_NB
//         const Eigen::Vector3d gE = frames::common::gECEF(pE_BE);         // radial m/s^2

//         const Eigen::Matrix3d CEB = CNB * CEN0;                          // E -> B
//         const Eigen::Vector3d gB = CEB * gE;

//         const Eigen::Vector3d FgB = mass * gB;
//         const Eigen::Vector3d FcounterB = -FgB;
//         const Eigen::Vector3d aB = (FgB + FcounterB) / mass;             // = 0

//         // 3) Translational dynamics in body
//         const Eigen::Vector3d vB_dot = aB - wB_BN.cross(vB_EB);
//         vB_EB += vB_dot * dt;

//         // 4) Translational kinematics in NED
//         const Eigen::Vector3d vN_EB = CNB.transpose() * vB_EB;            // v^N = C_BN v^B
//         pN_BN += vN_EB * dt;

//         // 5) Store back into the frame object if you want to use it elsewhere
//         BODY.HNB.set(CNB, pN_BN);
//     }

//     std::cout << "Final pN_BN: " << pN_BN.transpose() << "\n";
// }




int main() {
    
    // 1a
    Eigen::Vector3d xE_BE_t0(global::r_earth, 0, 0); // m
    Eigen::Vector3d xdotB_BE(150, 0, 0); // ms^-1

    double psi = global::deg2rad(0), theta = global::deg2rad(0), phi = global::deg2rad(0);
    Eigen::Matrix3d C_BN = transforms::eul2C_intr(psi, theta, phi, "ZYX");

    int tf = 1000; //s
    Eigen::Vector3d xE_BE_t;


    // 1a.i
    io::DataMatrix q1aiDM{ Eigen::MatrixXd::Zero(tf, 4) };

    double lat = global::deg2rad(0), lon = global::deg2rad(0);
    Eigen::Matrix3d C_NE = frames::CEN_from_lat_lon(lat, lon).transpose();
    Eigen::Matrix3d C_BE = transforms::chain_rot_extr(std::vector<Eigen::Matrix3d>{C_BN, C_NE});
    Eigen::Vector3d xdotE_BE = C_BE * xdotB_BE;

    // auto [xE_BE_tf, _] = dynamics::fwd_euler(xE_BE_t0, xdotE_BE, dynamics::common::f_cv, tf);
    // std::cout << xE_BE_tf << std::endl;
    // std::cout << std::endl;

    xE_BE_t = xE_BE_t0;
    for (int t = 0; t < tf; ++t){

        auto [xE_BE_t1, _] = dynamics::fwd_euler(xE_BE_t, xdotE_BE, dynamics::common::f_cv, 1);
        xE_BE_t = xE_BE_t1;

        q1aiDM.data(t, 0) = static_cast<double>(t);
        q1aiDM.data.block<1, 3>(t, 1) = xE_BE_t1.transpose();
    }

    q1aiDM.save("data/hw3", "q1ai");
    std::cout << xE_BE_t << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;



    // 1a.ii
    io::DataMatrix q1aiiDM{ Eigen::MatrixXd::Zero(tf, 4) };

    double lat_t = global::deg2rad(0), lon_t = global::deg2rad(0);

    Eigen::Matrix3d C_NE_t;
    Eigen::Matrix3d C_BE_t;
    xE_BE_t = xE_BE_t0;
    Eigen::Vector3d xdotB_BE_t = xdotB_BE;
    Eigen::Vector3d xdotE_BE_t;

    for (int t = 0; t < tf; ++t){
        auto [lat_t, lon_t, alt] = frames::lat_lon_alt_from_xECEF(xE_BE_t);

        C_NE_t = frames::CEN_from_lat_lon(lat_t, lon_t).transpose();
        C_BE_t = transforms::chain_rot_extr(std::vector<Eigen::Matrix3d>{C_BN, C_NE_t});
        xdotE_BE_t = C_BE_t * xdotB_BE_t;

        auto [xE_BE_t1, _] = dynamics::fwd_euler(xE_BE_t, xdotE_BE_t, dynamics::common::f_cv, 1);

        xE_BE_t = xE_BE_t1;

        q1aiiDM.data(t, 0) = static_cast<double>(t);
        q1aiiDM.data.block<1, 3>(t, 1) = xE_BE_t1.transpose();
    }

    q1aiiDM.save("data/hw3", "q1aii");
    std::cout << xE_BE_t << std::endl;
    std::cout << std::endl;
    std::cout << std::endl;


    // 2a
    q2(true);
    q2(false);


}