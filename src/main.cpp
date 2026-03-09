#include <Eigen/Dense>
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>
#include "simulation/transforms/transforms.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/global/global.hpp"
#include "simulation/frames/frames.hpp"
#include "simulation/vehicles/vehicles.hpp"
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "core/io/io.hpp"
#include "core/connection/connection.hpp"
#include "core/messages/messages.hpp"


void run(vehicles::Aircraft& asw28){

    // define StepOptions
    vehicles::StepOptions StepOpts;

    // get aircraft properties
    structural::StructuralProperties structuralProperties = asw28.structuralProperties;
    aerodynamics::AerodynamicProperties aerodynamicProperties = asw28.aerodynamicProperties;

    // declare and define useful variables
    dynamics::Mass mass = structuralProperties.Mass;
    dynamics::InertiaTensor J = structuralProperties.J;

    // define no external moment 
    Eigen::Vector3d MB_ext = global::Zero3;
    dynamics::Force FB_net;
    dynamics::Moment MB_net;

    // initialize rigid body state
    dynamics::RigidBodyState xN_t = asw28.rigidBodyState(asw28.FRDFrameNED);

    // run for 10 seconds
    const int tf = static_cast<int>(500.0 / global::dt);

    // create data matrix
    // io::DataMatrix positionDM{ Eigen::MatrixXd::Zero(tf, 4) };
    // io::DataMatrix aeroForceDM{ Eigen::MatrixXd::Zero(tf, 4) };
    // io::DataMatrix aeroMomentDM{ Eigen::MatrixXd::Zero(tf, 4) };
    // io::DataMatrix aeroStateDM{ Eigen::MatrixXd::Zero(tf, 4) };

    // initialize udp connections
    connection::UDPOut udp_out(5510);
    connection::UDPIn udp_in("127.0.0.1", 5511);

    // start timer
    using clock = std::chrono::steady_clock;
    auto next = clock::now();

    for (int t = 0; t < tf; ++t) {

        // step timer by dt
        next += std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(global::dt)
        );

        // define external force to cancel gravity
        Eigen::Vector3d FB_g = mass.data * asw28.FRDFrameNED.gB.data;
        Eigen::Vector3d FB_ext = global::Zero3; //-FB_g;    

        // compute density at current altitude
        atmospheric::Density rho = asw28.atmosphericState(asw28.FRDFrameECEF).rho;

        // compute aerodynamics forces and moments
        dynamics::Wrench WB_aero = step_aero_forces_moments(aerodynamicProperties, structuralProperties, xN_t, rho);
        dynamics::Force FB_aero = WB_aero.F;
        dynamics::Moment MB_aero = WB_aero.M;

        // compute net forces and moments
        FB_net.data = FB_ext + FB_g + FB_aero.data;
        MB_net.data = MB_ext + MB_aero.data;

        // step the simulation
        xN_t = dynamics::step_rigid_body(xN_t, mass, J, FB_net, MB_net);
        StepOpts.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions{ .rbs_BN = xN_t };
        asw28.step(StepOpts);

        // save data
        // positionDM.set(t, xN_t.p.data, global::dt);
        // aeroForceDM.set(t, FB_aero.data, global::dt);
        // aeroMomentDM.set(t, MB_aero.data, global::dt);
        // aerodynamics::AerodynamicState ads = asw28.aerodynamicState(asw28.FRDFrameNED);
        // aeroStateDM.set(t, Eigen::Vector3d(ads.Vinf.data, ads.alpha.data, ads.beta.data), global::dt);

        if (auto out_pkt = udp_out.try_receive()) {
            // use out_pkt->wind_heading, out_pkt->wind_speed
        }

        // fill in_pkt from the simulation state
        geography::GeographicState geo_state = asw28.geographicState(asw28.FRDFrameECEF);
        dynamics::EulerAngles att = asw28.FRDFrameNED.eulNB;
        messages::FlightGearMessageIn in_pkt{
            .altitude = global::m_to_ft(geo_state.alt.data),
            .latitude = global::rad_to_deg(geo_state.lat.data),
            .longitude = global::rad_to_deg(geo_state.lon.data),
            .roll = global::rad_to_deg(att.phi()),
            .pitch = global::rad_to_deg(att.theta()),
            .heading = global::rad_to_deg(att.psi()),
        };

        // send packet
        udp_in.send(in_pkt);

        // sleep to maintain frequency dictated by dt
        std::this_thread::sleep_until(next);

        // print state for debugging
        asw28.print_state(t);
    }

    // write data to csv
    // positionDM.write_csv("data/test/position", "position");
    // aeroForceDM.write_csv("data/test/aero_force", "aero_force");
    // aeroMomentDM.write_csv("data/test/aero_moment", "aero_moment");
    // aeroStateDM.write_csv("data/test/aero_state", "aero_state");
}






int main() {
    
    // // define structural and geometric properties
    // std::vector<structural::Geometry> geometries = {
    //     { .id = "s4",    .mass =  0.090, .x_size = 0.1,   .y_size = 0.96,  .z_size = 0.01,  .x_loc = -0.23,  .y_loc = 0.44,  .z_loc =  0.0  },  // Right Wing + Servo (s4)
    //     { .id = "s5",    .mass =  0.090, .x_size = 0.1,   .y_size = 0.96,  .z_size = 0.01,  .x_loc = -0.23,  .y_loc = -0.44, .z_loc =  0.0  },  // Left Wing + Servo (s5)
    //     { .id = "s2",    .mass =  0.013, .x_size = 0.075, .y_size = 0.35,  .z_size = 0.002, .x_loc = -0.76,  .y_loc = 0.0,   .z_loc = -0.16 },  // Horizontal Stabilizer (s2)
    //     { .id = "s3",    .mass =  0.000, .x_size = 0.08,  .y_size = 0.002, .z_size = 0.18,  .x_loc = -0.76,  .y_loc = 0.0,   .z_loc = -0.09 },  // Vertical Stabilizer (s3)
    //     { .id = "bat",   .mass =  0.072, .x_size = 0.065, .y_size = 0.035, .z_size = 0.015, .x_loc = -0.05,  .y_loc = 0.0,   .z_loc =  0.03 },  // Battery
    //     { .id = "fus",   .mass =  0.106, .x_size = 0.87,  .y_size = 0.07,  .z_size = 0.07,  .x_loc = -0.4,   .y_loc = 0.0,   .z_loc =  0.0  },  // Fuselage
    //     { .id = "mctrl", .mass =  0.027, .x_size = 0.05,  .y_size = 0.03,  .z_size = 0.005, .x_loc = -0.05,  .y_loc = 0.0,   .z_loc =  0.02 },  // Motor Controller
    //     { .id = "rad",   .mass =  0.010, .x_size = 0.04,  .y_size = 0.02,  .z_size = 0.005, .x_loc = -0.1,   .y_loc = 0.0,   .z_loc =  0.02 },  // Radio
    //     { .id = "serv",  .mass =  0.020, .x_size = 0.05,  .y_size = 0.01,  .z_size = 0.01,  .x_loc = -0.014, .y_loc = 0.0,   .z_loc =  0.0  },  // 2 Servos
    //     { .id = "mot",   .mass =  0.040, .x_size = 0.03,  .y_size = 0.02,  .z_size = 0.02,  .x_loc =  0.02,  .y_loc = 0.0,   .z_loc =  0.01 },  // Motor
    //     { .id = "prop",  .mass =  0.012, .x_size = 0.0,   .y_size = 0.26,  .z_size = 0.025, .x_loc =  0.05,  .y_loc = 0.0,   .z_loc =  0.01 }   // Propeller
    // };
    // structural::StructuralProperties structuralProperties(geometries);

    // // define aerodynamic properties
    // std::vector<aerodynamics::Surface> surfaces = {
    //     { .id = "s4", .chord = 0.10,  .span = 0.96, .p_ref = Eigen::Vector3d(-0.23,  0.44,  0.0),  .n = Eigen::Vector3d(0.0, 0.0, -1.0), .CL0 = 0.05, .e = 0.9, .i = 0.05, .CD0 = 0.01, .CDa = 1.0, .a0 = 0.05, .CM0 = -0.05, .CMa = 0.0 },  // Right Wing (s4)
    //     { .id = "s5", .chord = 0.10,  .span = 0.96, .p_ref = Eigen::Vector3d(-0.23, -0.44,  0.0),  .n = Eigen::Vector3d(0.0, 0.0, -1.0), .CL0 = 0.05, .e = 0.9, .i = 0.05, .CD0 = 0.01, .CDa = 1.0, .a0 = 0.05, .CM0 = -0.05, .CMa = 0.0 },  // Left Wing (s5)
    //     { .id = "s2", .chord = 0.075, .span = 0.35, .p_ref = Eigen::Vector3d(-0.76,  0.0,  -0.16), .n = Eigen::Vector3d(0.0, 0.0, -1.0), .CL0 = 0.0,  .e = 0.8, .i = 0.0,  .CD0 = 0.01, .CDa = 1.0, .a0 = 0.0,  .CM0 = 0.0,   .CMa = 0.0 },  // Horizontal Stabilizer (s2) 
    //     { .id = "s3", .chord = 0.08,  .span = 0.18, .p_ref = Eigen::Vector3d(-0.76,  0.0,  -0.09), .n = Eigen::Vector3d(0.0, 1.0,  0.0), .CL0 = 0.0,  .e = 0.8, .i = 0.0,  .CD0 = 0.01, .CDa = 1.0, .a0 = 0.0,  .CM0 = 0.0,   .CMa = 0.0 },  // Vertical Stabilizer (s3)
    // };
    // aerodynamics::AerodynamicProperties aerodynamicProperties(surfaces);

    // define structural and geometric properties
    std::vector<structural::Geometry> geometries = {
        { .id = "s4",    .mass = 45.00, .x_size = 0.700, .y_size = 7.500, .z_size = 0.120, .x_loc = -1.740, .y_loc =  3.44, .z_loc =  0.000 },  // Right Wing + Servo (s4)
        { .id = "s5",    .mass = 45.00, .x_size = 0.700, .y_size = 7.500, .z_size = 0.120, .x_loc = -1.740, .y_loc = -3.44, .z_loc =  0.000 },  // Left Wing + Servo (s5)
        { .id = "s2",    .mass =  8.00, .x_size = 0.600, .y_size = 2.600, .z_size = 0.100, .x_loc = -5.940, .y_loc =  0.00, .z_loc = -1.250 },  // Horizontal Stabilizer (s2)
        { .id = "s3",    .mass =  6.00, .x_size = 0.900, .y_size = 0.120, .z_size = 1.350, .x_loc = -5.940, .y_loc =  0.00, .z_loc = -0.700 },  // Vertical Stabilizer (s3)
        { .id = "bat",   .mass =  8.00, .x_size = 0.250, .y_size = 0.120, .z_size = 0.100, .x_loc = -0.380, .y_loc =  0.00, .z_loc =  0.200 },  // Battery
        { .id = "fus",   .mass = 131.0, .x_size = 6.580, .y_size = 0.650, .z_size = 1.100, .x_loc = -3.030, .y_loc =  0.00, .z_loc =  0.000 },  // Fuselage
        { .id = "mctrl", .mass =  5.00, .x_size = 0.250, .y_size = 0.180, .z_size = 0.080, .x_loc = -0.380, .y_loc =  0.00, .z_loc =  0.150 },  // Motor Controller
        { .id = "rad",   .mass =  3.00, .x_size = 0.180, .y_size = 0.100, .z_size = 0.060, .x_loc = -0.760, .y_loc =  0.00, .z_loc =  0.150 },  // Radio
        { .id = "serv",  .mass =  4.00, .x_size = 0.250, .y_size = 0.080, .z_size = 0.080, .x_loc = -0.110, .y_loc =  0.00, .z_loc =  0.000 },  // 2 Servos
        { .id = "mot",   .mass =  0.00, .x_size = 0.000, .y_size = 0.000, .z_size = 0.000, .x_loc =  0.000, .y_loc =  0.00, .z_loc =  0.000 },  // Motor
        { .id = "prop",  .mass =  0.00, .x_size = 0.000, .y_size = 0.000, .z_size = 0.000, .x_loc =  0.000, .y_loc =  0.00, .z_loc =  0.000 }   // Propeller
    };
    structural::StructuralProperties structuralProperties(geometries);

    // define aerodynamic properties
    std::vector<aerodynamics::Surface> surfaces = {
        { .id = "s4", .chord = 0.7, .span = 7.50, .p_ref = Eigen::Vector3d(-1.74,  3.44,  0.00), .n = Eigen::Vector3d(0.0, 0.0, -1.0), .CL0 = 0.2, .e = 0.95, .i = 0.03, .CD0 = 0.012, .CDa = 1.0, .a0 = -0.03, .CM0 = -0.08, .CMa = 0.0 },  // Right Wing (s4)
        { .id = "s5", .chord = 0.7, .span = 7.50, .p_ref = Eigen::Vector3d(-1.74, -3.44,  0.00), .n = Eigen::Vector3d(0.0, 0.0, -1.0), .CL0 = 0.2, .e = 0.95, .i = 0.03, .CD0 = 0.012, .CDa = 1.0, .a0 = -0.03, .CM0 = -0.08, .CMa = 0.0 },  // Left Wing (s5)
        { .id = "s2", .chord = 0.6, .span = 2.60, .p_ref = Eigen::Vector3d(-5.94,  0.00, -1.25), .n = Eigen::Vector3d(0.0, 0.0, -1.0), .CL0 = 0.0, .e = 0.90, .i = 0.00, .CD0 = 0.010, .CDa = 1.0, .a0 = 0.000, .CM0 = 0.000, .CMa = 0.0 },  // Horizontal Stabilizer (s2) 
        { .id = "s3", .chord = 0.9, .span = 1.35, .p_ref = Eigen::Vector3d(-5.94,  0.00, -0.70), .n = Eigen::Vector3d(0.0, 1.0,  0.0), .CL0 = 0.0, .e = 0.90, .i = 0.00, .CD0 = 0.010, .CDa = 1.0, .a0 = 0.000, .CM0 = 0.000, .CMa = 0.0 },  // Vertical Stabilizer (s3)
    };
    aerodynamics::AerodynamicProperties aerodynamicProperties(surfaces);

    // create vehicle
    vehicles::Aircraft asw28{ 
        structuralProperties, 
        aerodynamicProperties,
    };

    // define initial NED state
    geography::Latitude lat_0{ global::deg_to_rad(64.0) };
    geography::Longitude lon_0{ global::deg_to_rad(-22.0) };
    geography::Altitude alt_0{ 0.0 };
    vehicles::NEDFrameECEFStepOptions NEDFrameECEFStepOpts = {
        .lat_NE = lat_0,
        .lon_NE = lon_0,
        .alt_NE = alt_0
    };

    // define initial FRD state
    dynamics::OrientationMatrix CNB_0{ global::I3 };
    dynamics::Position pN_BN_0{ Eigen::Vector3d(0.0, 0.0, -750.0) };
    dynamics::AngularVelocity wB_BN{ Eigen::Vector3d(0.0, 0.0, 0.0) };
    dynamics::LinearVelocity vB_BN_0{ Eigen::Vector3d(50.0, 0.0, 0.0) };
    vehicles::FRDFrameNEDStepOptions FRDFrameNEDStepOpts = {
        .CNB = CNB_0,
        .pN_BN = pN_BN_0,
        .wB_BN = wB_BN,
        .vB_BN = vB_BN_0
    };

    // initialize simulation
    vehicles::StepOptions StepOpts = {
        .NEDFrameECEFStepOpts = NEDFrameECEFStepOpts,
        .FRDFrameNEDStepOpts = FRDFrameNEDStepOpts,
    };
    asw28.step(StepOpts);

    // Run case
    run(asw28);
}