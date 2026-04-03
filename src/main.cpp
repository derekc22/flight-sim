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
#include "simulation/autopilot/autopilot.hpp"
#include "simulation/analysis/analysis.hpp"
#include "core/io/io.hpp"
#include "core/connection/connection.hpp"
#include "core/messages/messages.hpp"


struct SimulationInput {
    vehicles::Aircraft& aircraft;
    double time_sec;
    bool trim_bool;
    bool verbose_bool;
    bool data_bool;
    std::string out_dir;
}; 
struct SimulationOutput {
    std::optional<io::DataMatrix> p_DM;
    std::optional<io::DataMatrix> eul_DM;
    std::optional<io::DataMatrix> w_DM;
    std::optional<io::DataMatrix> v_DM;
    autopilot::TrimSolution trim_sol;
    analysis::TrimLinearization lin_sol;
    analysis::TrimEigenAnalysis eig_sol;
}; 


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


void cleanup(const SimulationInput& sim_in, const SimulationOutput& sim_out) {
    std::string out_dir_path = "data/" + sim_in.out_dir + "/";
    if (sim_in.data_bool){
        // save data
        sim_out.p_DM->write_csv(out_dir_path, "p");
        sim_out.eul_DM->write_csv(out_dir_path, "eul");
        sim_out.w_DM->write_csv(out_dir_path, "w");
        sim_out.v_DM->write_csv(out_dir_path, "v");
        
        // log trim
        if (sim_in.trim_bool){
            io::write_txt(autopilot::print_trim_solution(sim_out.trim_sol), out_dir_path, "trim_sol");
            
            // log linearization and eigenanalysis
            if (sim_out.trim_sol.converged) {
                io::write_txt(analysis::print_linearization_solution(sim_out.lin_sol), out_dir_path, "lin_sol");
                io::write_txt(analysis::print_eigen_analysis(sim_out.eig_sol), out_dir_path, "eig_sol");
            }
        }

        // dump configs
        io::dump_configs(out_dir_path);
    }
}


void run(SimulationInput& sim_in, SimulationOutput& sim_out) {
    // unpack aircraft object
    vehicles::Aircraft& aircraft = sim_in.aircraft;

    // get aircraft properties
    structural::StructuralProperties structural_properties = aircraft.structural_properties;
    aerodynamics::AerodynamicProperties aerodynamic_properties = aircraft.aerodynamic_properties;
    control::ControlProperties control_properties = aircraft.control_properties;

    dynamics::Mass mass = structural_properties.Mass;
    dynamics::InertiaTensor J = structural_properties.J;

    autopilot::TrimSolution trim_sol;

    // run for user-specified seconds
    const int tf = std::max(1, static_cast<int>(std::ceil(sim_in.time_sec / constants::dt)));

    if (sim_in.data_bool) {
        sim_out.p_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.eul_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.w_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.v_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
    }

    // initialize udp connections
    connection::UDPOut udp_out(5510);
    connection::UDPIn udp_in("127.0.0.1", 5511);

    // start timer
    using clock = std::chrono::steady_clock;
    auto next = clock::now();

    for (int t = 0; t < tf; ++t) {

        // get rigid body state
        dynamics::RigidBodyState xN_t = aircraft.rigidBodyState(aircraft.FRDFrameNED);

        // step timer by dt
        next += std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(constants::dt));

        // compute airdensity at current altitude
        atmospheric::AirDensity rho = aircraft.staticAtmosphericState(aircraft.FRDFrameECEF).rho;

        // fetch wind
        // if (auto out_pkt = udp_out.try_receive()) {
        //     // use out_pkt->wind_heading, out_pkt->wind_speed
        // }
        atmospheric::Wind wind{ constants::Zero3 }; // no wind for now

        // specify control commands
        control::ControlSurfaceInputs u{};
        if (trim_sol.converged) { u.elevator = trim_sol.input.elevator; u.aileron = trim_sol.input.aileron; u.rudder = trim_sol.input.rudder; }

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

        // trim and linearization
        if (sim_in.trim_bool && !trim_sol.attempted) {
            trim_sol = autopilot::inspect_trim(aircraft, wind);
            sim_out.trim_sol = trim_sol;

            if (trim_sol.converged){
                // obtain full state from trim solution
                auto [xN_t_trim, ads_trim] = autopilot::update_state_from_trim(xN_t, trim_sol);

                // define TrimStepOptions
                vehicles::StepOptions TrimStepOptions;

                // overwrite state with trim state
                TrimStepOptions.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions { .rbs_BN = xN_t_trim };
                TrimStepOptions.STABFrameFRDStepOpts = vehicles::STABFrameFRDStepOptions{ .ads = ads_trim };
                TrimStepOptions.WINDFrameSTABStepOpts = vehicles::WINDFrameSTABStepOptions{ .ads = ads_trim };

                // step frames
                aircraft.step(TrimStepOptions);

                // overwrite local state with trim state
                xN_t = xN_t_trim;

                // linearize
                const analysis::TrimLinearization lin_sol = analysis::linearize_trim_solution(aircraft, trim_sol);
                const analysis::TrimEigenAnalysis eig_sol = analysis::trim_linearization_eigen_analysis(lin_sol);
                sim_out.lin_sol = lin_sol;
                sim_out.eig_sol = eig_sol;
            }            
        }

        // update data matrix
        if (sim_in.data_bool) {
            sim_out.p_DM->insert(t, xN_t.p.data, constants::dt);
            sim_out.eul_DM->insert(t, transforms::quatC_to_eul(xN_t.q.data, "ZYX", "intr"), constants::dt);
            sim_out.w_DM->insert(t, xN_t.w.data, constants::dt);
            sim_out.v_DM->insert(t, xN_t.v.data, constants::dt);
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

        // print state
        if (sim_in.verbose_bool) aircraft.print_state(t, wind);
    }

    // cleanup
    cleanup(sim_in, sim_out);
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
    std::string out_dir = argv[5];

    // load vehicle
    vehicles::Aircraft aircraft = load(trim_bool);

    // create simulation input
    SimulationInput sim_in { 
        .aircraft=aircraft, 
        .time_sec=time_sec,
        .trim_bool=trim_bool,
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
