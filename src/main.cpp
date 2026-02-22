#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "simulation/transforms/transforms.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/global/global.hpp"
#include "simulation/frames/frames.hpp"
#include "simulation/vehicles/vehicles.hpp"
#include "core/io/io.hpp"


void case1(vehicles::Aircraft&  ac, 
           frames::StepOptions& NEDFrameStepOptions, 
           frames::StepOptions& BODYFrameNEDStepOptions, 
           frames::StepOptions& BODYFrameECEFStepOptions){

    // Clear step options
    NEDFrameStepOptions.clear();
    BODYFrameNEDStepOptions.clear();
    BODYFrameECEFStepOptions.clear();
    
    // Define external force
    Eigen::Vector3d gB = ac.BODYFrameNED.gB.data;
    Eigen::Vector3d FB_g = ac.properties.mass.data * gB;
    Eigen::Vector3d FB_ext = -FB_g;
    dynamics::Force FB_net{ FB_ext + FB_g };

    std::cout << gB << std::endl;
    std::cout << std::endl;
    std::cout << FB_net.data << std::endl;

    // Define external moment 
    dynamics::Moment Mnet_B{ Eigen::Vector3d(0, 0, 0) };

    // Define rotation rate
    Eigen::Vector3d wB_BE(global::deg2rad(30), 0, 0); // rotates about x at 30˚/s

    BODYFrameECEFStepOptions = { .w = wB_BE };
    ac.BODYFrameECEF.step(BODYFrameECEFStepOptions);
    std::cout << ac.BODYFrameNED.HNB.p().data << std::endl;

    int tf = 6/dynamics::common::dt; // run for 6 seconds to complete 180˚ rotation about x 
    for (int t = 0; t < tf; ++t) {
        dynamics::RigidBodyState xE_t = ac.BODYFrameECEF.RigidBodyState();
        dynamics::RigidBodyState xE_t1 = dynamics::step_rigid_body(xE_t, ac.properties.mass, ac.properties.J, FB_net, Mnet_B);
        ac.BODYFrameECEF.step(xE_t1);

        std::cout << "t = " << t << std::endl;
        std::cout << xE_t1.pI_BI.data << std::endl;
        std::cout << std::endl;
        std::cout << xE_t1.vB_BI.data << std::endl;
        std::cout << std::endl;
        std::cout << xE_t1.qIB.data << std::endl;
        std::cout << std::endl;
        std::cout << xE_t1.wB_BI.data << std::endl;
        std::cout << "--------------------------"<< std::endl;
    }


}


int main() {
    
    // 6a

    // define frames
    frames::ECEFFrame ECEFFrame;
    frames::NEDFrameECEF NEDFrame;
    frames::FRDFrameECEF BODYFrameECEF;
    frames::FRDFrameNED BODYFrameNED;

    double lat_0 = 0.0, lon_0 = 0.0;
    Eigen::Matrix3d CEN_0 = frames::CEN_from_lat_lon(lat_0, lon_0);
    Eigen::Vector3d pE_NE_0(global::r_earth, 0, 0);

    Eigen::Matrix3d CNB_0 = global::I3;
    Eigen::Vector3d pN_BN_0(0, 10, 0);

    Eigen::Matrix3d CEB_0 = CNB_0 * CEN_0;
    Eigen::Vector3d pE_BE_0 = CEN_0.transpose() * pN_BN_0 + pE_NE_0;

    Eigen::Vector3d gE = frames::common::gECEF(pE_BE_0);
    Eigen::Vector3d gB = CEB_0 * gE;

    frames::StepOptions NEDFrameStepOptions{
        .C = CEN_0,
        .p = pE_NE_0,
    };
    frames::StepOptions BODYFrameNEDStepOptions{
        .C = CNB_0,
        .p = pN_BN_0,
        .g = gB
    };
    frames::StepOptions BODYFrameECEFrameStepOptions{
        .C = CEB_0,
        .p = pE_BE_0,
        .g = gE
    };

    NEDFrame.step(NEDFrameStepOptions);
    BODYFrameNED.step(BODYFrameNEDStepOptions);
    BODYFrameECEF.step(BODYFrameECEFrameStepOptions);

    // create vehicle
    dynamics::Mass mass { 1 }; // kg
    Eigen::Matrix3d J { Eigen::Matrix3d::Identity() };
    vehicles::Properties properties{ .mass = mass, .J = J };
    vehicles::Aircraft ac{ 
        .NEDFrame = NEDFrame, 
        .BODYFrameNED = BODYFrameNED, 
        .BODYFrameECEF = BODYFrameECEF, 
        .properties = properties 
    };

    // Case 1
    case1(ac, NEDFrameStepOptions, BODYFrameNEDStepOptions, BODYFrameECEFrameStepOptions);

}


// #include <Eigen/Dense>
// #include <iostream>
// #include <vector>

// #include "simulation/transforms/transforms.hpp"
// #include "simulation/dynamics/dynamics.hpp"
// #include "simulation/global/global.hpp"
// #include "simulation/frames/frames.hpp"
// #include "simulation/vehicles/vehicles.hpp"
// #include "core/io/io.hpp"

// // Helper: print quaternion as w x y z
// static void print_quat_wxyz(const Eigen::Quaterniond& q, const std::string& name) {
//     std::cout << name << " (w x y z) = "
//               << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << "\n";
// }

// // Helper: print matrix with label
// static void print_mat3(const Eigen::Matrix3d& M, const std::string& name) {
//     std::cout << name << " =\n" << M << "\n";
// }

// // Compute relative quaternion q_rel = q_final * conj(q_initial)
// // This is correct for the "q maps E->B" convention (passive, frame rotation).
// static Eigen::Quaterniond relative_q_E_to_B(const Eigen::Quaterniond& q_final,
//                                            const Eigen::Quaterniond& q_initial) {
//     Eigen::Quaterniond q_rel = q_final * q_initial.conjugate();
//     return transforms::normalize_and_canonicalize(q_rel);
// }

// void case1(vehicles::Aircraft&  ac,
//            frames::StepOptions& NEDFrameStepOptions,
//            frames::StepOptions& BODYFrameNEDStepOptions,
//            frames::StepOptions& BODYFrameECEFStepOptions) {

//     // Clear step options
//     NEDFrameStepOptions.clear();
//     BODYFrameNEDStepOptions.clear();
//     BODYFrameECEFStepOptions.clear();

//     // Net force (should be exactly zero by construction here)
//     Eigen::Vector3d gB = ac.BODYFrameNED.gB.data;
//     Eigen::Vector3d FB_g  = ac.properties.mass.data * gB;
//     Eigen::Vector3d FB_ext = -FB_g;
//     dynamics::Force FB_net{ FB_ext + FB_g };

//     // Net moment
//     dynamics::Moment Mnet_B{ Eigen::Vector3d(0, 0, 0) };

//     // Body angular rate about body x
//     Eigen::Vector3d wB_BE(global::deg2rad(30), 0, 0);

//     // Apply angular rate to BODYFrameECEF
//     BODYFrameECEFStepOptions = { .w = wB_BE };
//     ac.BODYFrameECEF.step(BODYFrameECEFStepOptions);

//     // Snapshot initial state BEFORE integration
//     const dynamics::RigidBodyState xE_0 = ac.BODYFrameECEF.RigidBodyState();
//     const Eigen::Quaterniond q0 = xE_0.qIB.data;

//     std::cout << "=== Initial diagnostics ===\n";
//     std::cout << "gB (from BODYFrameNED) =\n" << gB << "\n\n";
//     std::cout << "FB_net =\n" << FB_net.data << "\n\n";

//     std::cout << "BODYFrameNED HNB.p() (should match pN_BN_0) =\n"
//               << ac.BODYFrameNED.HNB.p().data << "\n\n";

//     std::cout << "Initial BODYFrameECEF position p (ECEF) =\n"
//               << xE_0.pI_BI.data << "\n\n";

//     std::cout << "Initial BODYFrameECEF w (body) =\n"
//               << xE_0.wB_BI.data << "\n\n";

//     print_quat_wxyz(q0, "q0");

//     // Also print the DCM implied by q0, and the frame's stored C
//     Eigen::Matrix3d C_from_q0 = transforms::quat2rot(q0);
//     print_mat3(C_from_q0, "C_from_q0 = quat2rot(q0)");
//     print_mat3(ac.BODYFrameECEF.HEB.C().data, "C_from_frame (HEB.C())");
//     print_mat3(ac.NEDFrame.HEN.C().data, "C_from_frame (HEN.C())");

//     std::cout << "===========================\n\n";

//     int tf = static_cast<int>(6.0 / dynamics::common::dt); // 6 seconds

//     dynamics::RigidBodyState xE_t1 = xE_0;

//     for (int t = 0; t < tf; ++t) {
//         dynamics::RigidBodyState xE_t = ac.BODYFrameECEF.RigidBodyState();
//         xE_t1 = dynamics::step_rigid_body(xE_t, ac.properties.mass, ac.properties.J, FB_net, Mnet_B);
//         ac.BODYFrameECEF.step(xE_t1);

//         // Keep your per-step prints if you want, but they are noisy.
//         // Print only final step by default.
//         if (t == tf - 1) {
//             std::cout << "=== Final step ===\n";
//             std::cout << "t = " << t << " (T = " << (t + 1) * dynamics::common::dt << " s)\n\n";

//             std::cout << "p =\n" << xE_t1.pI_BI.data << "\n\n";
//             std::cout << "vB =\n" << xE_t1.vB_BI.data << "\n\n";

//             const Eigen::Quaterniond qf = xE_t1.qIB.data;
//             print_quat_wxyz(qf, "qf");

//             // Relative rotation from initial to final (E->B convention)
//             Eigen::Quaterniond qrel = relative_q_E_to_B(qf, q0);
//             print_quat_wxyz(qrel, "qrel = qf * conj(q0)");

//             // Matrices from quaternions
//             Eigen::Matrix3d C_from_qf = transforms::quat2rot(qf);
//             Eigen::Matrix3d C_from_qrel = transforms::quat2rot(qrel);
//             print_mat3(C_from_qf, "C_from_qf = quat2rot(qf)");
//             print_mat3(C_from_qrel, "C_from_qrel = quat2rot(qrel)");

//             // Expected body-x 180 deg rotation matrix
//             Eigen::Matrix3d Rx_pi = Eigen::Matrix3d::Identity();
//             Rx_pi(1,1) = -1.0;
//             Rx_pi(2,2) = -1.0;
//             print_mat3(Rx_pi, "Expected Rx(pi) about BODY x");

//             // Compare numerically
//             std::cout << "||C_from_qrel - Rx(pi)||_F = "
//                       << (C_from_qrel - Rx_pi).norm() << "\n\n";

//             std::cout << "wB =\n" << xE_t1.wB_BI.data << "\n";
//             std::cout << "==================\n";
//         }
//     }
// }

// int main() {
//     // Frames
//     frames::ECEFFrame ECEFFrame;
//     frames::NEDFrameECEF NEDFrame;
//     frames::FRDFrameECEF BODYFrameECEF;
//     frames::FRDFrameNED BODYFrameNED;

//     double lat_0 = 0.0, lon_0 = 0.0;
//     Eigen::Matrix3d CEN_0 = frames::CEN_from_lat_lon(lat_0, lon_0);
//     Eigen::Vector3d pE_NE_0(global::r_earth, 0, 0);

//     Eigen::Matrix3d CNB_0 = global::I3;
//     Eigen::Vector3d pN_BN_0(0, 10, 0);

//     Eigen::Matrix3d CEB_0 = CNB_0 * CEN_0;
//     Eigen::Vector3d pE_BE_0 = CEN_0.transpose() * pN_BN_0 + pE_NE_0;

//     Eigen::Vector3d gE = frames::common::gECEF(pE_BE_0);
//     Eigen::Vector3d gB = CEB_0 * gE;

//     frames::StepOptions NEDFrameStepOptions{
//         .C = CEN_0,
//         .p = pE_NE_0,
//     };
//     frames::StepOptions BODYFrameNEDStepOptions{
//         .C = CNB_0,
//         .p = pN_BN_0,
//         .g = gB
//     };
//     frames::StepOptions BODYFrameECEFrameStepOptions{
//         .C = CEB_0,
//         .p = pE_BE_0,
//         .g = gE
//     };

//     NEDFrame.step(NEDFrameStepOptions);
//     BODYFrameNED.step(BODYFrameNEDStepOptions);
//     BODYFrameECEF.step(BODYFrameECEFrameStepOptions);

//     // Vehicle
//     dynamics::Mass mass { 1 };
//     Eigen::Matrix3d J { Eigen::Matrix3d::Identity() };
//     vehicles::Properties properties{ .mass = mass, .J = J };

//     vehicles::Aircraft ac{
//         .NEDFrame = NEDFrame,
//         .BODYFrameNED = BODYFrameNED,
//         .BODYFrameECEF = BODYFrameECEF,
//         .properties = properties
//     };

//     // Run case
//     case1(ac, NEDFrameStepOptions, BODYFrameNEDStepOptions, BODYFrameECEFrameStepOptions);
// }
