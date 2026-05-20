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
#include "simulation/transforms/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/util/public.hpp"
#include "simulation/frames/public.hpp"
#include "simulation/vehicles/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/structural/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/control/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/analysis/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/propulsion/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/estimation/public.hpp"
#include "simulation/avionics/public.hpp"
#include "simulation/operating/public.hpp"
#include "core/io/public.hpp"
#include "core/json/public.hpp"
#include "core/connection/public.hpp"
#include "core/messages/public.hpp"


struct SimulationInput {
    vehicles::Aircraft& aircraft;
    int tf;
    bool trim_bool;
    bool sensor_bool;
    bool control_bool;
    bool estimation_bool;
    bool wind_bool;
    bool verbose_bool;
    bool data_bool;
    std::string out_dir;
    io::DataManager data_manager;
};

struct SimulationOutput {
    trim::TrimSolution trim_sol;
    linearization::TrimLinearization lin_sol;
    analysis::TrimEigenAnalysis eig_sol;
};

struct SimulationContext {
    dynamics::RigidBodyState xN_t;
    dynamics::RigidBodyState yN_t;
    dynamics::RigidBodyState zN_t;
    dynamics::Wrench WB_net;
    atmospheric::StaticAtmosphericState static_atm_state;
    geography::GeographicState geographic_state;
    atmospheric::Wind windB;
    trim::TrimSolution trim_sol;
    linearization::TrimLinearization lin_sol;
};


vehicles::Aircraft load(bool trim_bool) {
    structural::StructuralProperties structural_properties = json::parse_structural_config();
    actuators::ActuatorProperties actuator_properties = json::parse_actuator_config(structural_properties);
    control::ControlProperties control_properties = json::parse_control_config();

    // create vehicle from config
    vehicles::Aircraft aircraft {
        structural_properties,
        json::parse_aerodynamics_config(),
        actuator_properties,
        control_properties,
        json::parse_avionics_config(),
        json::parse_operating_config(actuator_properties),
        json::parse_guidance_config(control_properties),
        json::parse_estimation_config()
    };

    // set initial conditions from config
    aircraft.step(json::parse_initialization_config(trim_bool));

    return aircraft;
}


void cleanup(SimulationInput& sim_in, SimulationOutput& sim_out) {
    std::string out_dir_path = "data/" + sim_in.out_dir + "/";

    // save data
    sim_in.data_manager.save(out_dir_path);

    // log trim
    if (sim_in.data_bool) {
        if (sim_in.trim_bool) {
            io::write_txt(trim::print_trim_solution(sim_out.trim_sol), out_dir_path, "trim_sol");

            // log linearization and eigenanalysis
            if (sim_out.trim_sol.converged) {
                io::write_txt(linearization::print_linearization_solution(sim_out.lin_sol), out_dir_path, "lin_sol");
                io::write_csv(Eigen::MatrixXd(sim_out.lin_sol.A), out_dir_path, "lin_sol_A");
                io::write_csv(Eigen::MatrixXd(sim_out.lin_sol.B), out_dir_path, "lin_sol_B");
                io::write_txt(analysis::print_eigen_analysis(sim_out.eig_sol), out_dir_path, "eig_sol");
            }
        }
    }

    // dump configs
    json::dump_configs(out_dir_path);
}


void run(SimulationInput& sim_in, SimulationOutput& sim_out) {
    // pack aircraft object
    vehicles::Aircraft& aircraft = sim_in.aircraft;

    // get aircraft properties
    structural::StructuralProperties& structural_properties = aircraft.structural_properties;
    aerodynamics::AerodynamicProperties& aerodynamic_properties = aircraft.aerodynamic_properties;
    control::ControlProperties& control_properties = aircraft.control_properties;
    estimation::EstimationProperties& estimation_properties = aircraft.estimation_properties;
    actuators::ActuatorProperties& actuator_properties = aircraft.actuator_properties;
    operating::OperatingProperties& operating_properties = aircraft.operating_properties;
    guidance::GuidanceProperties& guidance_properties = aircraft.guidance_properties;

    // extract reuseable quantities
    dynamics::Mass mass = structural_properties.Mass;
    dynamics::InertiaTensor J = structural_properties.J;
    
    actuators::SurfaceActuators& surface_actuators = actuator_properties.surface_actuators;
    actuators::PropulsorActuators& propulsor_actuators = actuator_properties.propulsor_actuators;

    actuators::SurfaceActuatorInputs_T<double> u_surface_actual_prev{};
    actuators::PropulsorActuatorInputs_T<double> u_propulsor_actual_prev{};

    // initialize trim and linearization solutions
    trim::TrimSolution trim_sol;
    linearization::TrimLinearization lin_sol;

    // initialize net force and moment
    dynamics::Force FB_net{ constants::Zero3 };
    dynamics::Moment MB_net{ constants::Zero3 };
    dynamics::Wrench WB_net{ .F = FB_net, .M = MB_net };

    // initialize windB
    atmospheric::Wind windB{ constants::Zero3 };

    // initialize udp connections
    connection::UDPOut udp_out(5510);
    connection::UDPIn udp_in("127.0.0.1", 5511);

    // start timer
    using clock = std::chrono::steady_clock;
    auto next = clock::now();

    for (int t = 0; t < sim_in.tf; ++t) {

        // get rigid body state
        dynamics::RigidBodyState xN_t = dynamics::compute_rigid_body_state(aircraft.FRDFrameNED);

        // compute static atmospheric state at current altitude
        atmospheric::StaticAtmosphericState static_atm_state = atmospheric::compute_static_atmospheric_state(aircraft.FRDFrameECEF);

        // fetch windB
        if (sim_in.wind_bool) {
            if (auto out_pkt = udp_out.try_receive()) { 
                atmospheric::Wind wind = atmospheric::build_wind(out_pkt->wind_heading, out_pkt->wind_speed);
                windB.data = frames::transform_vec(wind.data, aircraft.NEDFrameECEF, aircraft.FRDFrameNED);
            }
        }

        // trim and linearization
        if (sim_in.trim_bool && !trim_sol.attempted) {
            trim_sol = trim::inspect_trim(aircraft, windB);
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
                u_surface_actual_prev.elevator_cmd = trim_sol.input.elevator_cmd;
                u_surface_actual_prev.aileron_cmd = trim_sol.input.aileron_cmd;
                u_surface_actual_prev.rudder_cmd = trim_sol.input.rudder_cmd;
                u_propulsor_actual_prev.front_propulsor_cmd = trim_sol.input.front_propulsor_cmd;
                u_propulsor_actual_prev.left_propulsor_cmd = trim_sol.input.left_propulsor_cmd;
                u_propulsor_actual_prev.right_propulsor_cmd = trim_sol.input.right_propulsor_cmd;

                /** @deprecated */
                // overwrite actuator lag state with trim controls
                // trim::update_actuators_from_trim(surface_actuators, propulsor_actuators, trim_sol);

                // linearize
                lin_sol = linearization::linearize_trim_solution(aircraft, trim_sol);

                // perform eigenanalysis
                const analysis::TrimEigenAnalysis eig_sol = analysis::trim_linearization_eigen_analysis(lin_sol);

                sim_out.lin_sol = lin_sol;
                sim_out.eig_sol = eig_sol;
            }
        }

        // initialize measurements to ground truth
        dynamics::RigidBodyState yN_t = xN_t;

        // use sensors
        if (sim_in.sensor_bool){
            geography::GeographicState gps_t = geography::compute_geographic_state(aircraft.FRDFrameECEF);

            // obtain full state from sensors
            aerodynamics::AerodynamicState ads_t = aerodynamics::compute_aerodynamic_state(xN_t, windB);

            // overwrite local measurement state with sensor measurements
            yN_t = avionics::get_state_from_avionics(xN_t, ads_t, static_atm_state, gps_t, mass, windB, WB_net, aircraft.avionics_properties);
        }

        // initialize estimated state to measurements
        dynamics::RigidBodyState zN_t = yN_t;

        if (sim_in.estimation_bool) {
            estimation::EstimationInput estimation_input {
                .yN_t = yN_t,
                .estimator_input = estimation::KalmanFilterInput {
                    .yN_t = yN_t,
                    .lin_sol = lin_sol,
                    .trim_sol = trim_sol,
                    .u_surface_actual_prev = u_surface_actual_prev,
                    .u_propulsor_actual_prev = u_propulsor_actual_prev
                }
            };

            // overwrite local estimation state with estimator estimates
            zN_t = estimation_properties.step(estimation_input, sim_in.trim_bool).zN_t;
        }

        // specify guidance setpoint
        guidance::GuidanceSetpoint setpoint = guidance_properties.step(t, sim_in.tf);

        // specify control commands
        control::ControlOutput u_cmd{};

        if (sim_in.trim_bool && !sim_in.control_bool) {
            u_cmd = trim::set_control_inputs_from_trim(trim_sol);
        }

        if (sim_in.control_bool) {
            control::ControllerInput controller_input {
                .axial_controller_input = control::AxialControllerInput{ .zN_t = zN_t, .surface_actuators = surface_actuators, .setpoint = guidance::AxialSetpoint{ setpoint } },
                .velocity_controller_input = control::VelocityControllerInput{ .zN_t = zN_t, .propulsor_actuators = propulsor_actuators, .setpoint = guidance::VelocitySetpoint{ setpoint } },
                .linear_full_state_feedback_controller_input = control::LinearFullStateFeedbackControllerInput{ .zN_t = zN_t, .u_sol_trim = trim_sol.input, .surface_actuators = surface_actuators, .propulsor_actuators = propulsor_actuators, .A = lin_sol.A, .B = lin_sol.B, .setpoint = guidance::LinearFullStateFeedbackSetpoint{ setpoint } },
                .nonlinear_controller_input = control::NonlinearControllerInput{ .zN_t = zN_t, .surface_actuators = surface_actuators, .propulsor_actuators = propulsor_actuators, .setpoint = guidance::NonlinearSetpoint{ setpoint } }
            };
            u_cmd = control_properties.step(controller_input, sim_in.trim_bool);
        }

        u_cmd.surface_inputs.flap_cmd = operating_properties.fixed_actuator_inputs.flap;
        u_cmd.surface_inputs.spoiler_cmd = operating_properties.fixed_actuator_inputs.spoiler;

        // apply surface actuator dynamics
        actuators::SurfaceActuatorInputs_T<double> u_surface_actual = actuator_properties.step(u_cmd.surface_inputs);

        // apply propulsor dynamics
        actuators::PropulsorActuatorInputs_T<double> u_propulsor_actual = actuator_properties.step(u_cmd.propulsor_inputs);
        u_surface_actual_prev = u_surface_actual;
        u_propulsor_actual_prev = u_propulsor_actual;

        // compute aerodynamics forces and moments
        aerodynamics::AerodynamicWrench WB_aero = aerodynamics::step_aero_forces_moments(aerodynamic_properties, structural_properties, xN_t, static_atm_state, u_surface_actual, windB);
        dynamics::Force FB_aero = WB_aero.F;
        dynamics::Moment MB_aero = WB_aero.M;

        // compute propulsive forces and momments
        propulsion::PropulsiveWrench WB_propulsive = propulsion::step_propulsive_forces_moments(propulsor_actuators, xN_t, static_atm_state, u_propulsor_actual);
        dynamics::Force FB_propulsive = WB_propulsive.F;
        dynamics::Moment MB_propulsive = WB_propulsive.M;

        // compute gravitational force
        Eigen::Vector3d FB_g = mass.data * aircraft.FRDFrameNED.gB.data;

        // compute net forces and moments
        FB_net = dynamics::Force{ FB_g + FB_aero.data + FB_propulsive.data };
        MB_net = dynamics::Moment{ MB_aero.data + MB_propulsive.data };
        WB_net = dynamics::Wrench{ .F = FB_net, .M = MB_net };

        // compute rigid-body dynamics
        xN_t = dynamics::step_rigid_body(xN_t, mass, J, WB_net);

        // declare StepOptions
        vehicles::StepOptions StepOpts;

        // set step options
        StepOpts.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions{ .rbs_BN = xN_t };
        aerodynamics::AerodynamicState ads_t = aerodynamics::compute_aerodynamic_state(xN_t, windB);
        StepOpts.STABFrameFRDStepOpts = vehicles::STABFrameFRDStepOptions{ .ads = ads_t };
        StepOpts.WINDFrameSTABStepOpts = vehicles::WINDFrameSTABStepOptions{ .ads = ads_t };

        // step frames
        aircraft.step(StepOpts);

        // update data context
        io::DataContext data_context{
            .xN_t=xN_t,
            .yN_t=yN_t,
            .zN_t=zN_t,
            .u_surface_actual=u_surface_actual,
            .u_propulsor_actual=u_propulsor_actual,
            .WB_net=WB_net,
            .WB_aero=WB_aero,
            .WB_propulsive=WB_propulsive,
            .setpoint=setpoint,
            .windB=windB
        };

        // step data manager
        sim_in.data_manager.step(t, data_context);

        // generate in_pkt from the simulation state
        messages::FlightGearMessageIn in_pkt = messages::process_in_pkt(
            geography::compute_geographic_state(aircraft.FRDFrameECEF),
            aircraft.FRDFrameNED.eulNB
        );

        // send packet
        udp_in.send(in_pkt);

        // print state
        if (sim_in.verbose_bool) { aircraft.print_state(t, windB); }

        // step timer by dt
        next += std::chrono::duration_cast<clock::duration>(std::chrono::duration<double>(constants::dt));

        // sleep to maintain frequency dictated by dt
        std::this_thread::sleep_until(next);
    }

    // cleanup
    cleanup(sim_in, sim_out);
}





int main(int argc, char* argv[]) {
    if (argc != 10) { return 1; }

    double time_sec;
    try { time_sec = std::stod(argv[1]); }
    catch (const std::exception&) {std::cerr << "invalid TIME_SEC: " << argv[1] << std::endl; return 1; }
    if (!std::isfinite(time_sec) || time_sec <= 0.0) { std::cerr << "TIME_SEC must be > 0" << std::endl; return 1; }

    bool trim_bool = std::stoi(argv[2]) == 1;
    bool sensor_bool = std::stoi(argv[3]) == 1;
    bool control_bool = std::stoi(argv[4]) == 1;
    bool estimation_bool = std::stoi(argv[5]) == 1;
    bool wind_bool = std::stoi(argv[6]) == 1;
    bool verbose_bool = std::stoi(argv[7]) == 1;
    bool data_bool = std::stoi(argv[8]) == 1;
    std::string out_dir = argv[9];

    // load vehicle
    vehicles::Aircraft aircraft = load(trim_bool);

    // compute number of simulation steps
    int tf = std::max(1, static_cast<int>(std::ceil(time_sec / constants::dt)));

    // create data manager
    io::DataManager data_manager{tf, data_bool, control_bool, sensor_bool, estimation_bool, wind_bool};

    // create simulation input
    SimulationInput sim_in {
        .aircraft=aircraft,
        .tf=tf,
        .trim_bool=trim_bool,
        .sensor_bool=sensor_bool,
        .control_bool=control_bool,
        .estimation_bool=estimation_bool,
        .wind_bool=wind_bool,
        .verbose_bool=verbose_bool,
        .data_bool=data_bool,
        .out_dir=out_dir,
        .data_manager=data_manager
    };

    // declare simulation output
    SimulationOutput sim_out;

    // run case
    run(sim_in, sim_out);

    return 0;
}
