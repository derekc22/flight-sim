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
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"
#include "simulation/frames/frames.hpp"
#include "simulation/vehicles/vehicles.hpp"
#include "simulation/atmospheric/atmospheric.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/actuators/actuators.hpp"
#include "simulation/autopilot/autopilot.hpp"
#include "simulation/analysis/analysis.hpp"
#include "core/io/io.hpp"
#include "core/json/json.hpp"
#include "core/connection/connection.hpp"
#include "core/messages/messages.hpp"


struct SimulationInput {
    vehicles::Aircraft& aircraft;
    double time_sec;
    bool trim_bool;
    bool sensor_bool;
    bool control_bool;
    bool verbose_bool;
    bool data_bool;
    std::string out_dir;
}; 

struct SimulationOutput {
    std::optional<io::DataMatrix> p_DM;
    std::optional<io::DataMatrix> eul_DM;
    std::optional<io::DataMatrix> w_DM;
    std::optional<io::DataMatrix> v_DM;
    std::optional<io::DataMatrix> u_DM;
    std::optional<io::DataMatrix> F_DM;
    std::optional<io::DataMatrix> M_DM;
    autopilot::TrimSolution trim_sol;
    analysis::TrimLinearization lin_sol;
    analysis::TrimEigenAnalysis eig_sol;
}; 


vehicles::Aircraft load(bool trim_bool) {
    // create vehicle from config
    vehicles::Aircraft aircraft { 
        json::parse_structural_config(), 
        json::parse_aerodynamics_config(),
        json::parse_actuator_config(),
        json::parse_control_config(),
        json::parse_avionics_config()
    };

    // set initial conditions from config
    aircraft.step(json::parse_initialization_config(trim_bool));

    return aircraft;
}


void cleanup(const SimulationInput& sim_in, const SimulationOutput& sim_out) {
    std::string out_dir_path = "data/" + sim_in.out_dir + "/";
    if (sim_in.data_bool){
        // save data
        io::write_csv(sim_out.p_DM->data, out_dir_path, "p");
        io::write_csv(sim_out.eul_DM->data, out_dir_path, "eul");
        io::write_csv(sim_out.w_DM->data, out_dir_path, "w");
        io::write_csv(sim_out.v_DM->data, out_dir_path, "v");
        io::write_csv(sim_out.u_DM->data, out_dir_path, "u");
        io::write_csv(sim_out.F_DM->data, out_dir_path, "F");
        io::write_csv(sim_out.M_DM->data, out_dir_path, "M");
        
        // log trim
        if (sim_in.trim_bool){
            io::write_txt(autopilot::print_trim_solution(sim_out.trim_sol), out_dir_path, "trim_sol");
            
            // log linearization and eigenanalysis
            if (sim_out.trim_sol.converged) {
                io::write_txt(analysis::print_linearization_solution(sim_out.lin_sol), out_dir_path, "lin_sol");
                io::write_csv(Eigen::MatrixXd(sim_out.lin_sol.A), out_dir_path, "lin_sol_A");
                io::write_csv(Eigen::MatrixXd(sim_out.lin_sol.B), out_dir_path, "lin_sol_B");
                io::write_txt(analysis::print_eigen_analysis(sim_out.eig_sol), out_dir_path, "eig_sol");
            }
        }

        // dump configs
        json::dump_configs(out_dir_path);
    }
}


void run(SimulationInput& sim_in, SimulationOutput& sim_out) {
    // unpack aircraft object
    vehicles::Aircraft& aircraft = sim_in.aircraft;

    // get aircraft properties
    structural::StructuralProperties structural_properties = aircraft.structural_properties;
    aerodynamics::AerodynamicProperties aerodynamic_properties = aircraft.aerodynamic_properties;
    actuators::ActuatorProperties actuator_properties = aircraft.actuator_properties;
    control::ControlProperties& control_properties = aircraft.control_properties;

    dynamics::Mass mass = structural_properties.Mass;
    dynamics::InertiaTensor J = structural_properties.J;

    // intialize trim solution
    autopilot::TrimSolution trim_sol;

    // initialize net forces
    dynamics::Force FB_net{ constants::Zero3 };
    dynamics::Moment MB_net{ constants::Zero3 };

    // run for user-specified seconds
    const int tf = std::max(1, static_cast<int>(std::ceil(sim_in.time_sec / constants::dt)));

    if (sim_in.data_bool) {
        sim_out.p_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.eul_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.w_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.v_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.u_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 5+1) };
        sim_out.F_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.M_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
    }

    // initialize udp connections
    connection::UDPOut udp_out(5510);
    connection::UDPIn udp_in("127.0.0.1", 5511);

    // start timer
    using clock = std::chrono::steady_clock;
    auto next = clock::now();

    for (int t = 0; t < tf; ++t) {

        // set wrench 
        dynamics::Wrench WB_net { .F=FB_net, .M=MB_net };

        // get rigid body state
        dynamics::RigidBodyState xN_t = dynamics::rigid_body_state(aircraft.FRDFrameNED);

        // initialize measurements to ground truth
        dynamics::RigidBodyState xN_meas_t = xN_t;

        // step timer by dt
        next += std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(constants::dt));

        // compute static atmospheric state at current altitude
        atmospheric::StaticAtmosphericState static_atmospheric_state = atmospheric::static_atmospheric_state(aircraft.FRDFrameECEF);

        // fetch wind
        // if (auto out_pkt = udp_out.try_receive()) {
        //     // use out_pkt->wind_heading, out_pkt->wind_speed
        // }
        atmospheric::Wind wind{ constants::Zero3 }; // no wind for now

        // use sensors 
        if (sim_in.sensor_bool){
            geography::GeographicState geo_t = geography::geographic_state(aircraft.FRDFrameECEF);

            // obtain full state from sensors
            aerodynamics::AerodynamicState ads_t = aerodynamics::compute_aerodynamic_state(xN_t, wind);
            auto [xN_t_sensor, ads_t_sensor] = avionics::get_state_from_avionics(xN_t, ads_t, static_atmospheric_state, geo_t, mass, wind, WB_net, aircraft.avionics_properties);

            // overwrite local measurement state with sensor measurements
            xN_meas_t = xN_t_sensor;
        }

        // specify control commands
        control::ControlSurfaceInputs u_cmd{};
        if (sim_in.trim_bool && trim_sol.converged) { 
            u_cmd.elevator = trim_sol.input.elevator; 
            u_cmd.aileron = trim_sol.input.aileron; 
            u_cmd.rudder = trim_sol.input.rudder; 
        }
        else if (sim_in.control_bool) {
            control::ControlSurfaceInputs u_ctrl = control_properties.step(xN_meas_t, actuator_properties.actuators);
            u_cmd.elevator = u_ctrl.elevator;
            u_cmd.aileron = u_ctrl.aileron;
            u_cmd.rudder = u_ctrl.rudder;
        }

        // apply actuator dynamics
        control::ControlSurfaceInputs u_actual = aircraft.actuator_properties.step(u_cmd);

        // compute aerodynamics forces and moments
        /** @deprecated */
        // aerodynamics::AerodynamicLoad LB_aero = step_aero_forces_moments(aerodynamic_properties, structural_properties, xN_t, static_atmospheric_state, u, actuator_properties, wind);
        aerodynamics::AerodynamicLoad LB_aero = step_aero_forces_moments(aerodynamic_properties, structural_properties, xN_t, static_atmospheric_state, u_actual, wind);
        dynamics::Force FB_aero = LB_aero.F;
        dynamics::Moment MB_aero = LB_aero.M;

        // compute gravitational force
        Eigen::Vector3d FB_g = mass.data * aircraft.FRDFrameNED.gB.data;

        // compute net forces and moments
        FB_net = dynamics::Force{ FB_g + FB_aero.data };
        MB_net = dynamics::Moment{ MB_aero.data };

        // compute rigid-body dynamics
        xN_t = dynamics::step_rigid_body(xN_t, mass, J, FB_net, MB_net);

        // define StepOptions
        vehicles::StepOptions StepOpts;

        // set step options
        StepOpts.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions{ .rbs_BN = xN_t };
        aerodynamics::AerodynamicState ads_t = aerodynamics::compute_aerodynamic_state(xN_t, wind);
        StepOpts.STABFrameFRDStepOpts = vehicles::STABFrameFRDStepOptions{ .ads = ads_t };
        StepOpts.WINDFrameSTABStepOpts = vehicles::WINDFrameSTABStepOptions{ .ads = ads_t };

        // step frames
        aircraft.step(StepOpts);

        // trim and linearization
        if (sim_in.trim_bool && !trim_sol.attempted) {
            trim_sol = autopilot::inspect_trim(aircraft, wind);
            sim_out.trim_sol = trim_sol;

            if (trim_sol.converged){
                // obtain full state from trim solution
                auto [xN_t_trim, ads_t_trim] = autopilot::update_state_from_trim(xN_t, trim_sol);

                dynamics::Wrench WB_net_trim = trim_sol.wrench;

                // define TrimStepOptions
                vehicles::StepOptions TrimStepOptions;

                // overwrite state with trim state
                TrimStepOptions.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions { .rbs_BN = xN_t_trim };
                TrimStepOptions.STABFrameFRDStepOpts = vehicles::STABFrameFRDStepOptions{ .ads = ads_t_trim };
                TrimStepOptions.WINDFrameSTABStepOpts = vehicles::WINDFrameSTABStepOptions{ .ads = ads_t_trim };

                // step frames
                aircraft.step(TrimStepOptions);

                // overwrite local state with trim state
                xN_t = xN_t_trim;
                WB_net = WB_net_trim;

                // overwrite actuator lag state with trim controls
                autopilot::update_actuators_from_trim(aircraft.actuator_properties.actuators, trim_sol);

                // linearize
                const analysis::TrimLinearization lin_sol = analysis::linearize_trim_solution(aircraft, trim_sol);
                const analysis::TrimEigenAnalysis eig_sol = analysis::trim_linearization_eigen_analysis(lin_sol);
                sim_out.lin_sol = lin_sol;
                sim_out.eig_sol = eig_sol;
            }            
        }

        // update data matrix
        if (sim_in.data_bool) {
            dynamics::EulerAngles eul_meas_t;
            eul_meas_t.set(xN_meas_t.q);
            Eigen::VectorXd u_t(5);
            u_t << u_actual.elevator, u_actual.aileron, u_actual.rudder, u_actual.flaps, u_actual.spoilers;

            sim_out.p_DM->insert(t, xN_meas_t.p.data);
            sim_out.eul_DM->insert(t, eul_meas_t.data);
            sim_out.w_DM->insert(t, xN_meas_t.w.data);
            sim_out.v_DM->insert(t, xN_meas_t.v.data);
            sim_out.u_DM->insert(t, u_t);
            sim_out.F_DM->insert(t, FB_net.data);
            sim_out.M_DM->insert(t, MB_net.data);
        }

        // generate in_pkt from the simulation state
        messages::FlightGearMessageIn in_pkt = messages::process_in_pkt(
            geography::geographic_state(aircraft.FRDFrameECEF), 
            aircraft.FRDFrameNED.eulNB
        );

        // send packet
        udp_in.send(in_pkt);

        // sleep to maintain frequency dictated by dt
        std::this_thread::sleep_until(next);

        // print state
        if (sim_in.verbose_bool) aircraft.print_state(t, wind);
    }

    // cleanup
    cleanup(sim_in, sim_out);
}





int main(int argc, char* argv[]) {
    if (argc != 8) { return 1; }
    
    double time_sec;
    try { time_sec = std::stod(argv[1]); } 
    catch (const std::exception&) {std::cerr << "invalid TIME_SEC: " << argv[1] << std::endl; return 1; }
    if (!std::isfinite(time_sec) || time_sec <= 0.0) { std::cerr << "TIME_SEC must be > 0" << std::endl; return 1; }

    bool trim_bool = std::stoi(argv[2]) == 1;
    bool sensor_bool = std::stoi(argv[3]) == 1;
    bool control_bool = std::stoi(argv[4]) == 1;
    bool verbose_bool = std::stoi(argv[5]) == 1;
    bool data_bool = std::stoi(argv[6]) == 1;
    std::string out_dir = argv[7];

    // load vehicle
    vehicles::Aircraft aircraft = load(trim_bool);

    // create simulation input
    SimulationInput sim_in { 
        .aircraft=aircraft, 
        .time_sec=time_sec,
        .trim_bool=trim_bool,
        .sensor_bool=sensor_bool,
        .control_bool=control_bool,
        .verbose_bool=verbose_bool,
        .data_bool=data_bool, 
        .out_dir=out_dir  
    };

    // declare simulation output
    SimulationOutput sim_out;

    // run case
    run(sim_in, sim_out);

    return 0;
}
