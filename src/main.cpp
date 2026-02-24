// #include <Eigen/Dense>
// #include <iostream>
// #include <vector>
// #include "simulation/transforms/transforms.hpp"
// #include "simulation/dynamics/dynamics.hpp"
// #include "simulation/global/global.hpp"
// #include "simulation/frames/frames.hpp"
// #include "simulation/vehicles/vehicles.hpp"
// #include "core/io/io.hpp"
// #include "simulation/atmospheric/atmospheric.hpp"
// #include "simulation/structural/structural.hpp"
// #include "simulation/aerodynamics/aerodynamics.hpp"


// void case1(vehicles::Aircraft&  asw28, 
//            frames::StepOptions& NEDFrameStepOptions, 
//            frames::StepOptions& BODYFrameNEDStepOptions, 
//            frames::StepOptions& BODYFrameECEFStepOptions){
    
//     // std::cout << T << std::endl;
//     // std::cout << rho << std::endl;
//     // std::cout << mu << std::endl;

//     // std::cout << gB << std::endl;
//     // std::cout << std::endl;
//     // std::cout << FB_net.data << std::endl;
//     // std::cout << asw28.structural.Mass.data << std::endl;
//     // std::cout << asw28.structural.CG.data << std::endl;
//     // std::cout << asw28.structural.J.data << std::endl;

//     // const auto& s3 = asw28.aerodynamic.surfaces[
//     //     asw28.aerodynamic.surfaceIDs.at("s3")
//     // ];

//     // std::cout << "chord: " << s3.chord << std::endl;
//     // std::cout << "span: "  << s3.span  << std::endl;
//     // std::cout << "p_loc: " << s3.p_ref.transpose() << std::endl;
//     // std::cout << "n: "     << s3.n.transpose()     << std::endl;
//     // std::cout << "CL0: " << s3.CL0 << std::endl;
//     // std::cout << "e: "   << s3.e   << std::endl;
//     // std::cout << "i: "   << s3.i   << std::endl;
//     // std::cout << "CD0: " << s3.CD0 << std::endl;
//     // std::cout << "CDa: " << s3.CDa << std::endl;
//     // std::cout << "a0: "  << s3.a0  << std::endl;
//     // std::cout << "CM0: " << s3.CM0 << std::endl;
//     // std::cout << "CMa: " << s3.CMa << std::endl;
//     // std::cout << "area: " << s3.area << std::endl;
//     // std::cout << "AR: "   << s3.AR   << std::endl;
//     // std::cout << "p_ac: " << s3.p_ac.transpose() << std::endl;

//     // Define external moment 
//     Eigen::Vector3d MB_ext = Eigen::Vector3d(0, 0, 0);

//     // Define rotation rate
//     Eigen::Vector3d wB_BE(global::deg2rad(30), 0, 0); // rotates about x at 30˚/s

//     // Define a velocity
//     Eigen::Vector3d vB_BE(10, 0, 0); // 10 m/s forward

//     BODYFrameECEFStepOptions = { .w = wB_BE, .v = vB_BE };
//     asw28.BODYFrameECEF.step(BODYFrameECEFStepOptions);
//     // std::cout << asw28.BODYFrameNED.HNB.p().data << std::endl;

//     dynamics::RigidBodyState xE_t;
//     dynamics::RigidBodyState xE_t1;
//     structural::StructuralProperties asw28_structural = asw28.structural;
//     aerodynamics::AerodynamicProperties asw28_aerodynamic = asw28.aerodynamic;

//     double altitude;
//     atmospheric::StandardAtmosphere stdAtm;

//     dynamics::Force FB_net;
//     dynamics::Moment MB_net;

//     Eigen::Vector3d gE;
//     Eigen::Vector3d gB;
//     Eigen::Vector3d FB_g;
//     Eigen::Vector3d FB_ext;

//     Eigen::Vector3d pE_EB;
//     Eigen::Matrix3d CEB;

//     int tf = 6/dynamics::common::dt; // run for 6 seconds to complete 180˚ rotation about x 
//     for (int t = 0; t < tf; ++t) {

//         // Compute gravity at current position
//         pE_EB = asw28.BODYFrameECEF.HEB.p().data;
//         CEB = asw28.BODYFrameECEF.HEB.C().data;
//         gE = frames::common::gECEF(pE_EB);
//         gB = CEB * gE;

//         // Define external force to cancel gravity
//         FB_g = asw28_structural.Mass.data * gB;
//         FB_ext = -FB_g;

//         // Compute altitude
//         altitude = pE_EB.norm() - global::r_earth;
//         auto [T, rho, mu] = stdAtm.measure(altitude);

//         // Obtain current rigid body state
//         xE_t = asw28.BODYFrameECEF.RigidBodyState();

//         aerodynamics::AerodynamicState ads = aerodynamics::compute_aerodynamic_state(xE_t);
//         auto [FB_aero, MB_aero] = step_aero_forces_moments(asw28_aerodynamic, asw28_structural, xE_t, rho);


//         FB_net.data = FB_ext + FB_g + FB_aero.data;
//         MB_net.data = MB_ext + MB_aero.data;

//         xE_t1 = dynamics::step_rigid_body(xE_t, asw28_structural.Mass, asw28_structural.J, FB_net, MB_net);
//         asw28.BODYFrameECEF.step(xE_t1);


//         std::cout << "t = " << t << std::endl;
//         std::cout << xE_t1.pI_BI.data << std::endl;
//         std::cout << std::endl;
//         std::cout << xE_t1.vB_BI.data << std::endl;
//         std::cout << std::endl;
//         std::cout << xE_t1.qIB.data << std::endl;
//         std::cout << std::endl;
//         std::cout << xE_t1.wB_BI.data << std::endl;
//         std::cout << ads.Vinf << std::endl;
//         std::cout << xE_t1.vB_BI.data.norm() << std::endl;
//         std::cout << ads.alpha << std::endl;
//         std::cout << ads.beta << std::endl;
//         std::cout << "--------------------------"<< std::endl;
//     }




// }


// int main() {
    
//     // 6a

//     // define frames
//     frames::ECEFFrame ECEFFrame;
//     frames::NEDFrameECEF NEDFrame;
//     frames::FRDFrameECEF BODYFrameECEF;
//     frames::FRDFrameNED BODYFrameNED;

//     double lat_0 = 0.0, lon_0 = 0.0;
//     Eigen::Matrix3d CEN_0 = frames::CEN_from_lat_lon(lat_0, lon_0);
//     Eigen::Vector3d pE_NE_0(global::r_earth, 0, 0);

//     Eigen::Matrix3d CNB_0 = global::I3;
//     Eigen::Vector3d pN_BN_0(0, 0, -1000);

//     Eigen::Matrix3d CEB_0 = CNB_0 * CEN_0;
//     Eigen::Vector3d pE_BE_0 = CEN_0.transpose() * pN_BN_0 + pE_NE_0;

//     frames::StepOptions NEDFrameStepOptions{
//         .C = CEN_0,
//         .p = pE_NE_0,
//     };
//     frames::StepOptions BODYFrameNEDStepOptions{
//         .C = CNB_0,
//         .p = pN_BN_0,
//     };
//     frames::StepOptions BODYFrameECEFrameStepOptions{
//         .C = CEB_0,
//         .p = pE_BE_0,
//     };

//     NEDFrame.step(NEDFrameStepOptions);
//     BODYFrameNED.step(BODYFrameNEDStepOptions);
//     BODYFrameECEF.step(BODYFrameECEFrameStepOptions);

//     // define structural and geometric properties
//     std::vector<structural::Geometry> geometries = {
//         { .id = "s4",    .mass =  0.090, .x_size = 0.1,   .y_size = 0.96,  .z_size = 0.01,  .x_loc = -0.23,  .y_loc = 0.44,  .z_loc =  0.0  },  // Right Wing + Servo (s4)
//         { .id = "s5",    .mass =  0.090, .x_size = 0.1,   .y_size = 0.96,  .z_size = 0.01,  .x_loc = -0.23,  .y_loc = -0.44, .z_loc =  0.0  },  // Left Wing + Servo (s5)
//         { .id = "s2",    .mass =  0.013, .x_size = 0.075, .y_size = 0.35,  .z_size = 0.002, .x_loc = -0.76,  .y_loc = 0.0,   .z_loc = -0.16 },  // Horizontal Stabilizer (s2)
//         { .id = "s3",    .mass =  0.000, .x_size = 0.08,  .y_size = 0.002, .z_size = 0.18,  .x_loc = -0.76,  .y_loc = 0.0,   .z_loc = -0.09 },  // Vertical Stabilizer (s3)
//         { .id = "bat",   .mass =  0.072, .x_size = 0.065, .y_size = 0.035, .z_size = 0.015, .x_loc = -0.05,  .y_loc = 0.0,   .z_loc =  0.03 },  // Battery
//         { .id = "fus",   .mass =  0.106, .x_size = 0.87,  .y_size = 0.07,  .z_size = 0.07,  .x_loc = -0.4,   .y_loc = 0.0,   .z_loc =  0.0  },  // Fuselage
//         { .id = "mctrl", .mass =  0.027, .x_size = 0.05,  .y_size = 0.03,  .z_size = 0.005, .x_loc = -0.05,  .y_loc = 0.0,   .z_loc =  0.02 },  // Motor Controller
//         { .id = "rad",   .mass =  0.010, .x_size = 0.04,  .y_size = 0.02,  .z_size = 0.005, .x_loc = -0.1,   .y_loc = 0.0,   .z_loc =  0.02 },  // Radio
//         { .id = "serv",  .mass =  0.020, .x_size = 0.05,  .y_size = 0.01,  .z_size = 0.01,  .x_loc = -0.014, .y_loc = 0.0,   .z_loc =  0.0  },  // 2 Servos
//         { .id = "mot",   .mass =  0.040, .x_size = 0.03,  .y_size = 0.02,  .z_size = 0.02,  .x_loc =  0.02,  .y_loc = 0.0,   .z_loc =  0.01 },  // Motor
//         { .id = "prop",  .mass =  0.012, .x_size = 0.0,   .y_size = 0.26,  .z_size = 0.025, .x_loc =  0.05,  .y_loc = 0.0,   .z_loc =  0.01 }   // Propeller
//     };
//     structural::StructuralProperties structural(geometries);


//     // define aerodynamic properties
//     std::vector<aerodynamics::Surface> surfaces = {
//         { .id = "s4", .chord = 0.10,  .span = 0.96, .p_ref = Eigen::Vector3d(-0.23,  0.44,  0.0),  .n = Eigen::Vector3d(0.0, 0.0, -1.0), .CL0 = 0.05, .e = 0.9, .i = 0.05, .CD0 = 0.01, .CDa = 1.0, .a0 = 0.05, .CM0 = -0.05, .CMa = 0.0 },  // Right Wing (s4)
//         { .id = "s5", .chord = 0.10,  .span = 0.96, .p_ref = Eigen::Vector3d(-0.23, -0.44,  0.0),  .n = Eigen::Vector3d(0.0, 0.0, -1.0), .CL0 = 0.05, .e = 0.9, .i = 0.05, .CD0 = 0.01, .CDa = 1.0, .a0 = 0.05, .CM0 = -0.05, .CMa = 0.0 },  // Left Wing (s5)
//         { .id = "s2", .chord = 0.075, .span = 0.35, .p_ref = Eigen::Vector3d(-0.76,  0.0,  -0.16), .n = Eigen::Vector3d(0.0, 0.0, -1.0), .CL0 = 0.0,  .e = 0.8, .i = 0.0,  .CD0 = 0.01, .CDa = 1.0, .a0 = 0.0,  .CM0 = 0.0,   .CMa = 0.0 },  // Horizontal Stabilizer (s2) 
//         { .id = "s3", .chord = 0.08,  .span = 0.18, .p_ref = Eigen::Vector3d(-0.76,  0.0,  -0.09), .n = Eigen::Vector3d(0.0, 1.0,  0.0), .CL0 = 0.0,  .e = 0.8, .i = 0.0,  .CD0 = 0.01, .CDa = 1.0, .a0 = 0.0,  .CM0 = 0.0,   .CMa = 0.0 },  // Vertical Stabilizer (s3)
//     };
//     aerodynamics::AerodynamicProperties aerodynamic(surfaces);


//     // create vehicle
//     vehicles::Aircraft asw28{ 
//         .NEDFrame = NEDFrame, 
//         .BODYFrameNED = BODYFrameNED, 
//         .BODYFrameECEF = BODYFrameECEF, 
//         .structural = structural,
//         .aerodynamic = aerodynamic
//     };

//     // Case 1
//     case1(asw28, NEDFrameStepOptions, BODYFrameNEDStepOptions, BODYFrameECEFrameStepOptions);

// }