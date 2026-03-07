#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include "simulation/transforms/transforms.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/global/global.hpp"
#include "simulation/frames/frames.hpp"
#include "simulation/vehicles/vehicles.hpp"
#include "core/io/io.hpp"
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "core/io/io.hpp"


// void run(const vehicles::Aircraft&  ac){

//     // Copy aircraft and declare step options
//     vehicles::Aircraft asw28 = ac;
//     frames::SetOptions NEDFrameSetOptions;
//     frames::SetOptions BODYFrameNEDSetOptions;
//     frames::SetOptions BODYFrameECEFSetOptions;
    
//     // Define a velocity
//     Eigen::Vector3d vB_BE(10, 0, 0); // 10 m/s forward

//     // Define a rotation rate
//     Eigen::Vector3d wB_BE(global::deg2rad(30), 0, 0); // rotates about x at 30˚/s

//     // Set velocity and rotation rate
//     BODYFrameECEFSetOptions = { .w = wB_BE, .v = vB_BE };
//     asw28.BODYFrameECEF.set(BODYFrameECEFSetOptions);

//     // Declare and define useful variables
//     dynamics::RigidBodyState xE_t;
//     dynamics::RigidBodyState xE_t1;
//     structural::StructuralProperties structuralProperties = asw28.structuralProperties;
//     aerodynamics::AerodynamicProperties aerodynamicProperties = asw28.aerodynamicProperties;

//     double altitude;
//     atmospheric::StandardAtmosphere stdAtm;

//     // Define no external moment 
//     Eigen::Vector3d MB_ext = Eigen::Vector3d(0, 0, 0);

//     dynamics::Force FB_net;
//     dynamics::Moment MB_net;

//     Eigen::Vector3d gE;
//     Eigen::Vector3d gB;
//     Eigen::Vector3d FB_g;
//     Eigen::Vector3d FB_ext;

//     Eigen::Vector3d pE_EB;
//     Eigen::Matrix3d CEB;

//     aerodynamics::AerodynamicState ads;

//     // Run for 6 seconds
//     int tf = 6/dynamics::common::dt;

//     // Create data matrix
//     io::DataMatrix positionDM{ Eigen::MatrixXd::Zero(tf, 4) };
//     io::DataMatrix aeroForceDM{ Eigen::MatrixXd::Zero(tf, 4) };
//     io::DataMatrix aeroMomentDM{ Eigen::MatrixXd::Zero(tf, 4) };
//     io::DataMatrix aeroStateDM{ Eigen::MatrixXd::Zero(tf, 4) };

//     for (int t = 0; t < tf; ++t) {

//         // Compute gravity at current position
//         pE_EB = asw28.BODYFrameECEF.HEB.p().data;
//         CEB = asw28.BODYFrameECEF.HEB.C().data;
//         gE = frames::common::gECEF(pE_EB);
//         gB = CEB * gE;

//         // Define external force to cancel gravity
//         FB_g = structuralProperties.Mass.data * gB;
//         FB_ext = -FB_g;

//         // Compute altitude and density
//         altitude = pE_EB.norm() - global::r_earth;
//         auto [T, rho, mu] = stdAtm.measure(altitude);

//         // Compute aerodynamics forces and moments
//         auto [FB_aero, MB_aero] = step_aero_forces_moments(aerodynamicProperties, structuralProperties, xE_t, rho);

//         // Compute net forces and moments
//         FB_net.data = FB_ext + FB_g + FB_aero.data;
//         MB_net.data = MB_ext + MB_aero.data;

//         // Step the simulation
//         xE_t1 = dynamics::step_rigid_body(xE_t, structuralProperties.Mass, structuralProperties.J, FB_net, MB_net);
//         asw28.BODYFrameECEF.set(xE_t1);

//         // Save data
//         positionDM.set(t, xE_t1.p.data, dynamics::common::dt);
//         aeroForceDM.set(t, FB_aero.data, dynamics::common::dt);
//         aeroMomentDM.set(t, MB_aero.data, dynamics::common::dt);
//         aeroStateDM.set(t, Eigen::Vector3d(ads.Vinf, ads.alpha, ads.beta), dynamics::common::dt);
//     }

//     // Write data to csv
//     positionDM.write_csv("data/test/position", "position");
//     aeroForceDM.write_csv("data/test/aero_force", "aero_force");
//     aeroMomentDM.write_csv("data/test/aero_moment", "aero_moment");
//     aeroStateDM.write_csv("data/test/aero_state", "aero_state");
// }






int main() {}
    
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

//     frames::SetOptions NEDFrameSetOptions{
//         .C = CEN_0,
//         .p = pE_NE_0,
//     };
//     frames::SetOptions BODYFrameNEDSetOptions{
//         .C = CNB_0,
//         .p = pN_BN_0,
//     };
//     frames::SetOptions BODYFrameECEFrameSetOptions{
//         .C = CEB_0,
//         .p = pE_BE_0,
//     };

//     NEDFrame.set(NEDFrameSetOptions);
//     BODYFrameNED.set(BODYFrameNEDSetOptions);
//     BODYFrameECEF.set(BODYFrameECEFrameSetOptions);

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
//     structural::StructuralProperties structuralProperties(geometries);

//     // define aerodynamic properties
//     std::vector<aerodynamics::Surface> surfaces = {
//         { .id = "s4", .chord = 0.10,  .span = 0.96, .p_ref = Eigen::Vector3d(-0.23,  0.44,  0.0),  .n = Eigen::Vector3d(0.0, 0.0, -1.0), .CL0 = 0.05, .e = 0.9, .i = 0.05, .CD0 = 0.01, .CDa = 1.0, .a0 = 0.05, .CM0 = -0.05, .CMa = 0.0 },  // Right Wing (s4)
//         { .id = "s5", .chord = 0.10,  .span = 0.96, .p_ref = Eigen::Vector3d(-0.23, -0.44,  0.0),  .n = Eigen::Vector3d(0.0, 0.0, -1.0), .CL0 = 0.05, .e = 0.9, .i = 0.05, .CD0 = 0.01, .CDa = 1.0, .a0 = 0.05, .CM0 = -0.05, .CMa = 0.0 },  // Left Wing (s5)
//         { .id = "s2", .chord = 0.075, .span = 0.35, .p_ref = Eigen::Vector3d(-0.76,  0.0,  -0.16), .n = Eigen::Vector3d(0.0, 0.0, -1.0), .CL0 = 0.0,  .e = 0.8, .i = 0.0,  .CD0 = 0.01, .CDa = 1.0, .a0 = 0.0,  .CM0 = 0.0,   .CMa = 0.0 },  // Horizontal Stabilizer (s2) 
//         { .id = "s3", .chord = 0.08,  .span = 0.18, .p_ref = Eigen::Vector3d(-0.76,  0.0,  -0.09), .n = Eigen::Vector3d(0.0, 1.0,  0.0), .CL0 = 0.0,  .e = 0.8, .i = 0.0,  .CD0 = 0.01, .CDa = 1.0, .a0 = 0.0,  .CM0 = 0.0,   .CMa = 0.0 },  // Vertical Stabilizer (s3)
//     };
//     aerodynamics::AerodynamicProperties aerodynamicProperties(surfaces);


//     // Obtain initial rigid body state
//     dynamics::RigidBodyState RigidBodyState = BODYFrameECEF.RigidBodyState();

//     // Obtain initial aerodynamic state
//     aerodynamics::AerodynamicState aerodynamicState = aerodynamics::compute_aerodynamic_state(RigidBodyState);

//     // create vehicle
//     vehicles::Aircraft asw28{ 
//         .NEDFrame = NEDFrame, 
//         .BODYFrameNED = BODYFrameNED, 
//         .BODYFrameECEF = BODYFrameECEF, 
//         .structuralProperties = structuralProperties,
//         .aerodynamicProperties = aerodynamicProperties,
//         .RigidBodyState = RigidBodyState,
//         .aerodynamicState = aerodynamicState
//     };


//     // Structural
//     // std::cout << "Mass: " << asw28.structuralProperties.Mass.data << std::endl;
//     // std::cout << "CG: " << asw28.structuralProperties.CG.data << std::endl;
//     // std::cout << "J: " << asw28.structuralProperties.J.data << std::endl;


//     // Aerodynamic
//     // const auto& s3 = asw28.aerodynamicProperties.surfaces[
//     //     asw28.aerodynamicProperties.surfaceIDs.at("s3")
//     // ];
//     // std::cout << "s3, p_loc: " << s3.p_ref.transpose() << std::endl;
//     // std::cout << "s3, p_ac: " << s3.p_ac.transpose() << std::endl;
//     // std::cout << "s3, chord: " << s3.chord << std::endl;
//     // std::cout << "s3, span: "  << s3.span  << std::endl;
//     // std::cout << "s3, AR: "   << s3.AR   << std::endl;

//     // const auto& s4 = asw28.aerodynamicProperties.surfaces[
//     //     asw28.aerodynamicProperties.surfaceIDs.at("s4")
//     // ];
//     // std::cout << "s4, p_loc: " << s4.p_ref.transpose() << std::endl;
//     // std::cout << "s4, p_ac: " << s4.p_ac.transpose() << std::endl;
//     // std::cout << "s4, chord: " << s4.chord << std::endl;
//     // std::cout << "s4, span: "  << s4.span  << std::endl;
//     // std::cout << "s4, AR: "   << s4.AR   << std::endl;

//     // const auto& s5 = asw28.aerodynamicProperties.surfaces[
//     //     asw28.aerodynamicProperties.surfaceIDs.at("s5")
//     // ];
//     // std::cout << "s5, p_loc: " << s5.p_ref.transpose() << std::endl;
//     // std::cout << "s5, p_ac: " << s5.p_ac.transpose() << std::endl;
//     // std::cout << "s5, chord: " << s5.chord << std::endl;
//     // std::cout << "s5, span: "  << s5.span  << std::endl;
//     // std::cout << "s5, AR: "   << s5.AR   << std::endl;

//     // const auto& s2 = asw28.aerodynamicProperties.surfaces[
//     //     asw28.aerodynamicProperties.surfaceIDs.at("s2")
//     // ];
//     // std::cout << "s2, p_loc: " << s2.p_ref.transpose() << std::endl;
//     // std::cout << "s2, p_ac: " << s2.p_ac.transpose() << std::endl;
//     // std::cout << "s2, chord: " << s2.chord << std::endl;
//     // std::cout << "s2, span: "  << s2.span  << std::endl;
//     // std::cout << "s2, AR: "   << s2.AR   << std::endl;



//     // Run case
//     run(asw28);

// }