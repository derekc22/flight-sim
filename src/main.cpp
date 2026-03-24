#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <chrono>
#include <stdexcept>
#include <string>
#include <thread>
#include <optional>
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



vehicles::Aircraft load(bool trim_bool) {
    // create vehicle from config
    vehicles::Aircraft aircraft { 
        io::parse_structural_config(), 
        io::parse_aerodynamics_config(),
        io::parse_control_config(),
    };

    // set initial conditions from config
    aircraft.step(io::parse_init_options_config(trim_bool));

    return aircraft;
}
void run(vehicles::Aircraft& aircraft, double time_sec, bool trim_bool, bool verbose_bool, bool data_bool, std::string data_folder) {
    // get aircraft properties
    structural::StructuralProperties structural_properties = aircraft.structural_properties;
    aerodynamics::AerodynamicProperties aerodynamic_properties = aircraft.aerodynamic_properties;
    control::ControlProperties control_properties = aircraft.control_properties;

    // declare and define useful variables
    dynamics::Mass mass = structural_properties.Mass;
    dynamics::InertiaTensor J = structural_properties.J;

    // define no external moment 
    Eigen::Vector3d MB_ext = global::Zero3;;

    // initialize rigid body state
    dynamics::RigidBodyState xN_t = aircraft.rigidBodyState(aircraft.FRDFrameNED);

    autopilot::TrimSolution trim;
    bool trim_ready = false;

    // run for user-specified seconds
    const int tf = std::max(1, static_cast<int>(std::ceil(time_sec / global::dt)));

    // create data matrices
    std::optional<io::DataMatrix> p_DM;
    std::optional<io::DataMatrix> eul_DM;
    std::optional<io::DataMatrix> w_DM;
    std::optional<io::DataMatrix> v_DM;

    if (data_bool) {
        p_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        eul_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        w_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        v_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
    }

    // initialize udp connections
    connection::UDPOut udp_out(5510);
    connection::UDPIn udp_in("127.0.0.1", 5511);

    // start timer
    using clock = std::chrono::steady_clock;
    auto next = clock::now();

    for (int t = 0; t < tf; ++t) {

        // step timer by dt
        next += std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(global::dt));

        // compute density at current altitude
        atmospheric::Density rho = aircraft.atmosphericState(aircraft.FRDFrameECEF).rho;

        // fetch wind
        // if (auto out_pkt = udp_out.try_receive()) {
        //     // use out_pkt->wind_heading, out_pkt->wind_speed
        // }
        atmospheric::Wind wind{ global::Zero3 }; // no wind for now

        // Before trim is available, propagate one tick with neutral controls
        // After that, hold the trim solution's control deflections
        control::ControlSurfaceInputs u{};
        if (trim_ready) { u.elevator = trim.input.elevator; u.aileron = trim.input.aileron; u.rudder = trim.input.rudder; }

        // compute aerodynamics forces and moments
        aerodynamics::AerodynamicLoad LB_aero = step_aero_forces_moments(aerodynamic_properties, structural_properties, xN_t, rho, u, control_properties, wind);
        dynamics::Force FB_aero = LB_aero.F;
        dynamics::Moment MB_aero = LB_aero.M;

        // compute gravitational force
        Eigen::Vector3d FB_g = mass.data * aircraft.FRDFrameNED.gB.data;

        // compute net forces and moments
        dynamics::Force FB_net{ FB_g + FB_aero.data };
        dynamics::Moment MB_net{ MB_aero.data };

        // compute rigid-body dynamics
        xN_t = dynamics::step_rigid_body(xN_t, mass, J, FB_net, MB_net);

        // define StepOptions
        vehicles::StepOptions StepOpts;

        // set step options
        StepOpts.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions{ .rbs_BN = xN_t };

        aerodynamics::AerodynamicState ads = aerodynamics::compute_aerodynamic_state(xN_t, wind);
        StepOpts.STABFrameFRDStepOpts = vehicles::STABFrameFRDStepOptions{ .ads = ads };
        StepOpts.WINDFrameSTABStepOpts = vehicles::WINDFrameSTABStepOptions{ .ads = ads };

        // step frames
        aircraft.step(StepOpts);

        if (trim_bool && !trim_ready) {
            trim = autopilot::inspect_trim(aircraft, wind);
            if (!trim.converged) { std::cerr << "trim failed to converge; aborting run" << std::endl; return; }
            trim_ready = true;

            dynamics::EulerAngles eul_curr;
            eul_curr.set(xN_t.q);
            dynamics::EulerAngles eul_trim{ Eigen::Vector3d(eul_curr.psi(), trim.state.theta, trim.state.phi) };
            dynamics::OrientationQuaternion qNB_trim;
            qNB_trim.set(eul_trim);

            dynamics::RigidBodyState xN_t_trim = { 
                .p = xN_t.p, 
                .v = dynamics::LinearVelocity{ Eigen::Vector3d(trim.state.vx, trim.state.vy, trim.state.vz) },
                .q = qNB_trim,
                .w = dynamics::AngularVelocity{ Eigen::Vector3d(trim.state.p, trim.state.q, trim.state.r) },
            };

            StepOpts.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions { .rbs_BN = xN_t_trim };

            aerodynamics::AerodynamicState ads_trim = aerodynamics::compute_aerodynamic_state(xN_t_trim, wind);
            StepOpts.STABFrameFRDStepOpts = vehicles::STABFrameFRDStepOptions{ .ads = ads_trim };
            StepOpts.WINDFrameSTABStepOpts = vehicles::WINDFrameSTABStepOptions{ .ads = ads_trim };

            // step frames
            aircraft.step(StepOpts);

            xN_t = xN_t_trim;
        }

        // save data
        if (data_bool) {
            p_DM->set(t, xN_t.p.data, global::dt);
            eul_DM->set(t, transforms::quatC2eul(xN_t.q.data, "ZYX", "intr"), global::dt);
            w_DM->set(t, xN_t.w.data, global::dt);
            v_DM->set(t, xN_t.v.data, global::dt);
        }

        // generate in_pkt from the simulation state
        messages::FlightGearMessageIn in_pkt = messages::process_in_pkt(
            aircraft.geographicState(aircraft.FRDFrameECEF), 
            aircraft.FRDFrameNED.eulNB
        );

        // send packet
        udp_in.send(in_pkt);

        // sleep to maintain frequency dictated by dt
        std::this_thread::sleep_until(next);

        // print state for debugging
        if (verbose_bool) aircraft.print_state(t, wind);
    }

    // write data to csv
    if (data_bool) {
        std::string csv_path = "data/" + data_folder + "/";
        p_DM->write_csv(csv_path, "p");
        eul_DM->write_csv(csv_path, "eul");
        w_DM->write_csv(csv_path, "w");
        v_DM->write_csv(csv_path, "v");
    }
}





int main(int argc, char* argv[]) {
    if (argc != 6) { return 1; }
    
    double time_sec;
    try { time_sec = std::stod(argv[1]); } 
    catch (const std::exception&) {std::cerr << "invalid TIME_SEC: " << argv[1] << std::endl; return 1; }
    if (!std::isfinite(time_sec) || time_sec <= 0.0) { std::cerr << "TIME_SEC must be > 0" << std::endl; return 1; }

    bool trim_bool = std::stoi(argv[2]) == 1;
    bool verbose_bool = std::stoi(argv[3]) == 1;
    bool data_bool = std::stoi(argv[4]) == 1;
    std::string data_folder = argv[5];

    // load vehicle
    vehicles::Aircraft aircraft = load(trim_bool);

    // run case
    run(aircraft, time_sec, trim_bool, verbose_bool, data_bool, data_folder);

    return 0;
}
