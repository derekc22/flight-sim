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
#include "simulation/trim/trim.hpp"
#include "simulation/analysis/analysis.hpp"
#include "simulation/linearization/linearization.hpp"
#include "simulation/propulsion/propulsion.hpp"
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
    std::optional<io::DataMatrix> u_surface_DM;
    std::optional<io::DataMatrix> u_propulsor_DM;
    std::optional<io::DataMatrix> F_net_DM;
    std::optional<io::DataMatrix> M_net_DM;
    std::optional<io::DataMatrix> F_aero_DM;
    std::optional<io::DataMatrix> M_aero_DM;
    std::optional<io::DataMatrix> F_prop_DM;
    std::optional<io::DataMatrix> M_prop_DM;
    trim::TrimSolution trim_sol;
    linearization::TrimLinearization lin_sol;
    analysis::TrimEigenAnalysis eig_sol;
}; 


vehicles::Aircraft load(bool trim_bool) {
    // create vehicle from config
    vehicles::Aircraft aircraft { 
        json::parse_structural_config(), 
        json::parse_aerodynamics_config(),
        json::parse_actuator_config(),
        json::parse_control_config(),
        json::parse_avionics_config(),
        json::parse_operating_config()
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
        io::write_csv(sim_out.u_surface_DM->data, out_dir_path, "u_surface");
        io::write_csv(sim_out.u_propulsor_DM->data, out_dir_path, "u_propulsor");
        io::write_csv(sim_out.F_net_DM->data, out_dir_path, "F_net");
        io::write_csv(sim_out.M_net_DM->data, out_dir_path, "M_net");
        io::write_csv(sim_out.F_aero_DM->data, out_dir_path, "F_aero");
        io::write_csv(sim_out.M_aero_DM->data, out_dir_path, "M_aero");
        io::write_csv(sim_out.F_prop_DM->data, out_dir_path, "F_prop");
        io::write_csv(sim_out.M_prop_DM->data, out_dir_path, "M_prop");
        
        // log trim
        if (sim_in.trim_bool){
            io::write_txt(trim::print_trim_solution(sim_out.trim_sol), out_dir_path, "trim_sol");
            
            // log linearization and eigenanalysis
            if (sim_out.trim_sol.converged) {
                io::write_txt(linearization::print_linearization_solution(sim_out.lin_sol), out_dir_path, "lin_sol");
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
    structural::StructuralProperties& structural_properties = aircraft.structural_properties;
    aerodynamics::AerodynamicProperties& aerodynamic_properties = aircraft.aerodynamic_properties;
    control::ControlProperties& control_properties = aircraft.control_properties;
    actuators::ActuatorProperties& actuator_properties = aircraft.actuator_properties;
    operating::OperatingProperties& operating_properties = aircraft.operating_properties;

    dynamics::Mass mass = structural_properties.Mass;
    dynamics::InertiaTensor J = structural_properties.J;

    control::SurfaceActuatorInputs fixed_surface_inputs = operating::fixed_surface_actuator_inputs(operating_properties);

    // initialize trim and linearization solutions
    trim::TrimSolution trim_sol;
    linearization::TrimLinearization lin_sol;

    // initialize net force and moment
    dynamics::Force FB_net{ constants::Zero3 };
    dynamics::Moment MB_net{ constants::Zero3 };

    // run for user-specified seconds
    const int tf = std::max(1, static_cast<int>(std::ceil(sim_in.time_sec / constants::dt)));

    if (sim_in.data_bool) {
        sim_out.p_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.eul_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.w_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.v_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.u_surface_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 5+1) };
        sim_out.u_propulsor_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.F_net_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.M_net_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.F_aero_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.M_aero_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.F_prop_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
        sim_out.M_prop_DM = io::DataMatrix{ Eigen::MatrixXd::Zero(tf, 3+1) };
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
        if (auto out_pkt = udp_out.try_receive()) {
            // use out_pkt->wind_heading, out_pkt->wind_speed
        }
        atmospheric::Wind wind{ constants::Zero3 }; // no wind for now

        // use sensors 
        if (sim_in.sensor_bool){
            geography::GeographicState geo_t = geography::geographic_state(aircraft.FRDFrameECEF);

            // obtain full state from sensors
            aerodynamics::AerodynamicState ads_t = aerodynamics::compute_aerodynamic_state(xN_t, wind);
            dynamics::RigidBodyState xN_t_sensor = avionics::get_state_from_avionics(xN_t, ads_t, static_atmospheric_state, geo_t, mass, wind, WB_net, aircraft.avionics_properties);

            // overwrite local measurement state with sensor measurements
            xN_meas_t = xN_t_sensor;
        }

        // specify control commands
        control::ControlInputs u_cmd{};

        if (sim_in.trim_bool && trim_sol.converged && !sim_in.control_bool) {
            u_cmd = trim::update_control_inputs_from_trim(trim_sol);
        }
        else if (sim_in.control_bool && !sim_in.trim_bool) {
            u_cmd = control_properties.step(xN_meas_t, actuator_properties.surface_actuators);
        }
        else if (sim_in.control_bool && sim_in.trim_bool && trim_sol.converged) {
            u_cmd = control_properties.step(
                xN_meas_t,
                lin_sol,
                trim_sol.input,
                actuator_properties.surface_actuators,
                actuator_properties.propulsor_actuators
            );
        }

        u_cmd.surface_inputs.flap_cmd = fixed_surface_inputs.flap_cmd;
        u_cmd.surface_inputs.spoiler_cmd = fixed_surface_inputs.spoiler_cmd;

        // apply actuator dynamics
        control::SurfaceActuatorInputs u_surface_actual = actuator_properties.step(u_cmd.surface_inputs);

        // apply propulsor dynamics
        control::PropulsorActuatorInputs u_propulsor_actual = actuator_properties.step(u_cmd.propulsor_inputs);

        // compute aerodynamics forces and moments
        aerodynamics::AerodynamicWrench WB_aero = step_aero_forces_moments(aerodynamic_properties, structural_properties, xN_t, static_atmospheric_state, u_surface_actual, wind);
        dynamics::Force FB_aero = WB_aero.F;
        dynamics::Moment MB_aero = WB_aero.M;

        // compute propulsive forces and momments
        propulsion::PropulsiveWrench WB_propulsive = propulsion::step_propulsive_forces_moments(actuator_properties.propulsor_actuators, u_propulsor_actual);
        dynamics::Force FB_propulsive = WB_propulsive.F;
        dynamics::Moment MB_propulsive = WB_propulsive.M;

        // compute gravitational force
        Eigen::Vector3d FB_g = mass.data * aircraft.FRDFrameNED.gB.data;

        // compute net forces and moments
        FB_net = dynamics::Force{ FB_g + FB_aero.data + FB_propulsive.data };
        MB_net = dynamics::Moment{ MB_aero.data + MB_propulsive.data };

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
            trim_sol = trim::inspect_trim(aircraft, wind);
            sim_out.trim_sol = trim_sol;

            if (trim_sol.converged){
                // obtain full state from trim solution
                auto [xN_t_trim, ads_t_trim] = trim::update_state_from_trim(xN_t, trim_sol);

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
                trim::update_actuators_from_trim(actuator_properties.surface_actuators, actuator_properties.propulsor_actuators, trim_sol);

                // linearize
                lin_sol = linearization::linearize_trim_solution(aircraft, trim_sol);

                // perform eigenanalysis
                const analysis::TrimEigenAnalysis eig_sol = analysis::trim_linearization_eigen_analysis(lin_sol);
                
                sim_out.lin_sol = lin_sol;
                sim_out.eig_sol = eig_sol;
            }            
        }

        // update data matrix
        if (sim_in.data_bool) {
            dynamics::EulerAngles eul_meas_t;
            eul_meas_t.set(xN_meas_t.q);
            Eigen::VectorXd u_surface_t(5);
            Eigen::VectorXd u_propulsor_t(3);
            u_surface_t << u_surface_actual.elevator_cmd, u_surface_actual.aileron_cmd, u_surface_actual.rudder_cmd, u_surface_actual.flap_cmd, u_surface_actual.spoiler_cmd;
            u_propulsor_t << u_propulsor_actual.front_propulsor_cmd, u_propulsor_actual.left_propulsor_cmd, u_propulsor_actual.right_propulsor_cmd;

            sim_out.p_DM->insert(t, xN_meas_t.p.data);
            sim_out.eul_DM->insert(t, eul_meas_t.data);
            sim_out.w_DM->insert(t, xN_meas_t.w.data);
            sim_out.v_DM->insert(t, xN_meas_t.v.data);
            sim_out.u_surface_DM->insert(t, u_surface_t);
            sim_out.u_propulsor_DM->insert(t, u_propulsor_t);
            sim_out.F_net_DM->insert(t, FB_net.data);
            sim_out.M_net_DM->insert(t, MB_net.data);
            sim_out.F_aero_DM->insert(t, FB_aero.data);
            sim_out.M_aero_DM->insert(t, MB_aero.data);
            sim_out.F_prop_DM->insert(t, FB_propulsive.data);
            sim_out.M_prop_DM->insert(t, MB_propulsive.data);
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
        if (sim_in.verbose_bool) { aircraft.print_state(t, wind); }
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
