#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <chrono>
#include <stdexcept>
#include <thread>
#include "simulation/transforms/transforms.hpp"
#include "simulation/dynamics/dynamics.hpp"
#include "simulation/global/global.hpp"
#include "simulation/frames/frames.hpp"
#include "simulation/vehicles/vehicles.hpp"
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/autopilot/autopilot.hpp"
#include "core/io/io.hpp"
#include "core/connection/connection.hpp"
#include "core/messages/messages.hpp"



vehicles::Aircraft load() {
    // create vehicle from config
    vehicles::Aircraft aircraft { 
        io::parse_structural_config(), 
        io::parse_aerodynamics_config(),
        io::parse_control_config(),
    };

    // set initial conditions from config
    aircraft.step(io::parse_init_options_config());

    return aircraft;
}



void run(vehicles::Aircraft& aircraft, double time_sec) {
    // get aircraft properties
    structural::StructuralProperties structuralProperties = aircraft.structuralProperties;
    aerodynamics::AerodynamicProperties aerodynamicProperties = aircraft.aerodynamicProperties;
    control::ControlProperties controlProperties = aircraft.controlProperties;

    // declare and define useful variables
    dynamics::Mass mass = structuralProperties.Mass;
    dynamics::InertiaTensor J = structuralProperties.J;

    // define no external moment 
    Eigen::Vector3d MB_ext = global::Zero3;;

    // initialize rigid body state
    dynamics::RigidBodyState xN_t = aircraft.rigidBodyState(aircraft.FRDFrameNED);

    // run for user-specified seconds
    const int tf = std::max(1, static_cast<int>(std::ceil(time_sec / global::dt)));

    // create data matrix
    io::DataMatrix p_DM{ Eigen::MatrixXd::Zero(tf, 3+1) };
    io::DataMatrix eul_DM{ Eigen::MatrixXd::Zero(tf, 3+1) };
    io::DataMatrix w_DM{ Eigen::MatrixXd::Zero(tf, 3+1) };
    io::DataMatrix v_DM{ Eigen::MatrixXd::Zero(tf, 3+1) };

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
        Eigen::Vector3d FB_g = mass.data * aircraft.FRDFrameNED.gB.data;
        Eigen::Vector3d FB_ext = 0 * -FB_g;

        // compute density at current altitude
        atmospheric::Density rho = aircraft.atmosphericState(aircraft.FRDFrameECEF).rho;

        // fetch wind
        // if (auto out_pkt = udp_out.try_receive()) {
        //     // use out_pkt->wind_heading, out_pkt->wind_speed
        // }
        atmospheric::Wind wind{ global::Zero3 }; // no wind for now

        // define control input
        control::ControlSurfaceInputs u{ .aileron = 1, .elevator = -0.25 };

        // compute aerodynamics forces and moments
        aerodynamics::AerodynamicLoad LB_aero = step_aero_forces_moments(
            aerodynamicProperties,
            structuralProperties,
            xN_t,
            rho,
            u,
            controlProperties,
            wind
        );
        dynamics::Force FB_aero = LB_aero.F;
        dynamics::Moment MB_aero = LB_aero.M;

        // compute net forces and moments
        dynamics::Force FB_net{ FB_ext + FB_g + FB_aero.data };
        dynamics::Moment MB_net{ MB_ext + MB_aero.data };

        // compute rigid-body dynamics
        xN_t = dynamics::step_rigid_body(xN_t, mass, J, FB_net, MB_net);

        // define StepOptions
        vehicles::StepOptions StepOpts;

        // set step options
        StepOpts.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions{ .rbs_BN = xN_t };

        aerodynamics::AerodynamicState ads = aerodynamics::compute_aerodynamic_state(xN_t, wind);
        StepOpts.STABFrameFRDStepOpts = vehicles::STABFrameFRDStepOptions{ .ads = ads };

        // step frames
        aircraft.step(StepOpts);

        // save data
        p_DM.set(t, xN_t.p.data, global::dt);
        eul_DM.set(t, transforms::quatC2eul(xN_t.q.data, "ZYX", "intr"), global::dt);
        w_DM.set(t, xN_t.w.data, global::dt);
        v_DM.set(t, xN_t.v.data, global::dt);

        // fill in_pkt from the simulation state
        geography::GeographicState geo_state = aircraft.geographicState(aircraft.FRDFrameECEF);
        dynamics::EulerAngles att = aircraft.FRDFrameNED.eulNB;
        messages::FlightGearMessageIn in_pkt{
            .altitude = global::m_to_ft(geo_state.alt.data),
            .latitude = global::rad_to_deg(geo_state.lat.data),
            .longitude = global::rad_to_deg(geo_state.lon.data),
            .roll = global::rad_to_deg(att.phi()),
            .pitch = global::rad_to_deg(att.theta()),
            .heading = global::rad_to_deg(att.psi())
        };

        // send packet
        udp_in.send(in_pkt);

        // sleep to maintain frequency dictated by dt
        std::this_thread::sleep_until(next);

        // print state for debugging
        aircraft.print_state(t, wind);
    }

    // write data to csv
    p_DM.write_csv("data/example", "p");
    eul_DM.write_csv("data/example", "eul");
    w_DM.write_csv("data/example", "w");
    v_DM.write_csv("data/example", "v");
}





int main(int argc, char* argv[]) {
    if (argc != 2) { std::cerr << "usage: " << argv[0] << " <TIME_SEC>" << std::endl; return 1; }
    
    double time_sec = 0.0;
    try { time_sec = std::stod(argv[1]); } 
    catch (const std::exception&) {std::cerr << "invalid time: " << argv[1] << std::endl; return 1; }
    if (!std::isfinite(time_sec) || time_sec <= 0.0) { std::cerr << "TIME_SEC must be > 0" << std::endl; return 1;}

    // load vehicle
    vehicles::Aircraft aircraft = load();

    // run case
    run(aircraft, time_sec);

    // temporary casadi test
    // autopilot::test_casadi();

    return 0;
}
