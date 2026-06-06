#include <chrono>
#include <string>
#include <thread>
#include <optional>
#include <Eigen/Dense>
#include "simulation/runner/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/avionics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/control/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/estimation/public.hpp"
#include "simulation/frames/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/propulsion/public.hpp"
#include "simulation/structural/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/vehicles/public.hpp"
#include "core/connection/public.hpp"
#include "core/json/actuators/public.hpp"
#include "core/json/aerodynamics/public.hpp"
#include "core/json/avionics/public.hpp"
#include "core/json/control/public.hpp"
#include "core/json/estimation/public.hpp"
#include "core/json/guidance/public.hpp"
#include "core/json/initialization/public.hpp"
#include "core/json/operating/public.hpp"
#include "core/json/public.hpp"
#include "core/json/structural/public.hpp"
#include "core/messages/public.hpp"
#include "analysis/eigenanalysis/public.hpp"

namespace runner {

    vehicles::Aircraft load(const std::string& aircraft_id, bool trim_bool) {
        structural::StructuralProperties structural_properties = json::parse_structural_config();
        actuators::ActuatorProperties actuator_properties = json::parse_actuator_config(structural_properties);
        control::ControlProperties control_properties = json::parse_control_config();

        // create vehicle from config
        vehicles::Aircraft aircraft {
            aircraft_id,
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

    SimulationRunner::SimulationRunner(SimulationOptions options) : 
        options(options),
        // load vehicle
        aircraft(load(options.aircraft_id, options.trim_bool)),
        // create data manager
        data_manager(options.tf, options.data_bool, options.control_bool, options.sensor_bool, options.estimation_bool, options.wind_bool),
        // create analysis manager
        analysis_manager(options.data_bool, options.analysis_bool, options.trim_bool),
        FB_net{ constants::Zero3 },
        MB_net{ constants::Zero3 },
        WB_net{ .F = FB_net, .M = MB_net },
        windB{ constants::Zero3 },
        // initialize udp connections
        udp_out(5510),
        udp_in("127.0.0.1", 5511),
        // start timer
        next(std::chrono::steady_clock::now()) {}

    SimulationRunner::~SimulationRunner() = default;

    void SimulationRunner::cleanup() {
        std::string data_dir_path = options.data_dir_path;
        std::string report_dir_path = options.report_dir_path;

        // save data
        data_manager.save(data_dir_path);
        analysis_manager.save(data_dir_path, report_dir_path);

        // dump configs
        json::dump_configs(data_dir_path);
    }

    void SimulationRunner::run() {
        for (int t = 0; t < options.tf; ++t) {
            step(t);
        }

        // cleanup
        cleanup();
    }

    void SimulationRunner::step(int t) {
        // pack aircraft object
        vehicles::Aircraft& aircraft = this->aircraft;

        // get aircraft properties
        structural::StructuralProperties& structural_properties = aircraft.structural_properties;
        aerodynamics::AerodynamicProperties& aerodynamic_properties = aircraft.aerodynamic_properties;
        control::ControlProperties& control_properties = aircraft.control_properties;
        estimation::EstimationProperties& estimation_properties = aircraft.estimation_properties;
        actuators::ActuatorProperties& actuator_properties = aircraft.actuator_properties;
        operating::OperatingProperties& operating_properties = aircraft.operating_properties;
        guidance::GuidanceProperties& guidance_properties = aircraft.guidance_properties;

        // extract reuseable quantities
        dynamics::Mass mass = structural_properties.mass;
        dynamics::InertiaTensor J = structural_properties.J;
        
        actuators::SurfaceActuators& surface_actuators = actuator_properties.surface_actuators;
        actuators::PropulsorActuators& propulsor_actuators = actuator_properties.propulsor_actuators;

        // get rigid body state
        dynamics::RigidBodyState Xt = dynamics::compute_rigid_body_state(aircraft.FRDFrameNED);

        // compute static atmospheric state at current altitude
        atmospheric::StaticAtmosphericState static_atm_state_t = atmospheric::compute_static_atmospheric_state(aircraft.FRDFrameECEF);

        // fetch windB
        if (options.wind_bool) {
            if (auto out_pkt = udp_out.try_receive()) { 
                atmospheric::Wind wind = atmospheric::build_wind(out_pkt->wind_heading, out_pkt->wind_speed);
                windB.data = frames::transform_vec(wind.data, aircraft.NEDFrameECEF, aircraft.FRDFrameNED);
            }
        }

        // trim and linearization
        if (options.trim_bool && !trim_sol.attempted) {
            trim_sol = trim::inspect_trim(aircraft, windB);

            // initialize analysis context
            io::AnalysisContext analysis_context{
                .aircraft_id=aircraft.id
            };

            // update analysis context
            analysis_context.trim_sol = trim_sol;

            if (trim_sol.converged) {
                // obtain full state from trim solution
                auto [Xt_trim, aero_state_t_trim] = trim::update_state_from_trim(Xt, trim_sol);

                dynamics::Wrench WB_net_trim = trim_sol.wrench;

                // define TrimStepOptions
                vehicles::StepOptions TrimStepOptions;

                // overwrite state with trim state
                TrimStepOptions.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions { .rbs_BN = Xt_trim };
                TrimStepOptions.STABFrameFRDStepOpts = vehicles::STABFrameFRDStepOptions{ .ads = aero_state_t_trim };
                TrimStepOptions.WINDFrameSTABStepOpts = vehicles::WINDFrameSTABStepOptions{ .ads = aero_state_t_trim };

                // step frames
                aircraft.step(TrimStepOptions);

                // overwrite local state with trim state
                Xt = Xt_trim;
                WB_net = WB_net_trim;
                auto [u_surf_trim, u_prop_trim] = trim::update_actuators_from_trim(u_surface_actual_prev, u_propulsor_actual_prev, trim_sol);
                u_surface_actual_prev = u_surf_trim;
                u_propulsor_actual_prev = u_prop_trim;

                /** @deprecated */
                // overwrite actuator lag state with trim controls
                // trim::update_actuators_lag_from_trim(surface_actuators, propulsor_actuators, trim_sol);

                // linearize
                lin_sol = linearization::linearize_operating_point(aircraft, trim_sol.operating_point, trim_sol.conditions);

                // perform eigenanalysis
                analysis::EigenAnalysis eig_sol = analysis::linearization_eigen_analysis(lin_sol);

                // update analysis context
                analysis_context.lin_sol = lin_sol;
                analysis_context.eig_sol = eig_sol;
            }

            // step analysis manager
            analysis_manager.step(analysis_context);
        }

        // initialize measurements to ground truth
        dynamics::RigidBodyState Yt = Xt;

        // use sensors
        if (options.sensor_bool){
            geography::GeographicState geo_state_t = geography::compute_geographic_state(aircraft.FRDFrameECEF);

            // obtain full state from sensors
            aerodynamics::AerodynamicState aero_state_t = aerodynamics::compute_aerodynamic_state(Xt, windB);

            // overwrite local measurement state with sensor measurements
            Yt = avionics::get_state_from_avionics(Xt, aero_state_t, static_atm_state_t, geo_state_t, mass, windB, WB_net, aircraft.avionics_properties);
        }

        // initialize estimated state to measurements
        dynamics::RigidBodyState Zt = Yt;

        if (options.estimation_bool) {
            linearization::LocalLinearization estimator_lin_sol;
            operating::OperatingPoint estimator_operating_point;
            operating::OperatingConditions estimator_conditions;

            if (estimation_properties.extended_kalman_estimator_type == estimation::EstimatorType::ExtendedKalmanFilter) {
                dynamics::State_T<double> yt = dynamics::pack_state(Yt);
                actuators::ActuatorInputs_T<double> actuator_inputs = actuators::pack_actuator_inputs(u_surface_actual_prev, u_propulsor_actual_prev);

                estimator_operating_point = operating::OperatingPoint {
                    .state = yt,
                    .input = actuator_inputs
                };

                estimator_conditions = {
                    .static_atm_state = static_atm_state_t,
                    .windB = windB
                };

            } else if (estimation_properties.linear_kalman_estimator_type == estimation::EstimatorType::LinearKalmanFilter) {
                estimator_lin_sol = lin_sol;
                estimator_operating_point = trim_sol.operating_point;
            }

            estimation::EstimatorInputs estimator_inputs {
                .Yt = Yt,
                .linear_kalman_estimator_input = estimation::LinearKalmanEstimatorInput {
                    .Yt = Yt,
                    .operating_point = estimator_operating_point,
                    .lin_sol = estimator_lin_sol,
                    .u_surface_actual_prev = u_surface_actual_prev,
                    .u_propulsor_actual_prev = u_propulsor_actual_prev,
                },
                .extended_kalman_estimator_input = estimation::ExtendedKalmanEstimatorInput {
                    .Yt = Yt,
                    .operating_point = estimator_operating_point,
                    .u_surface_actual_prev = u_surface_actual_prev,
                    .u_propulsor_actual_prev = u_propulsor_actual_prev,
                    .aircraft = aircraft,
                    .conditions = estimator_conditions
                }
            };

            // overwrite local estimation state with estimator estimates
            Zt = estimation_properties.step(estimator_inputs, options.trim_bool).Zt;
        }

        // specify guidance setpoint
        guidance::GuidanceSetpoint setpoint = guidance_properties.step(t, options.tf);

        // specify control commands
        control::ControlOutput u_cmd{};

        if (options.trim_bool && !options.control_bool) {
            u_cmd = trim::set_control_inputs_from_trim(trim_sol);
        }

        if (options.control_bool) {
            control::ControllerInputs controller_inputs {
                .attitude_controller_input = control::AttitudeControllerInput{ .Zt = Zt, .surface_actuators = surface_actuators, .setpoint = guidance::AttitudeSetpoint{ setpoint } },
                .velocity_controller_input = control::VelocityControllerInput{ .Zt = Zt, .propulsor_actuators = propulsor_actuators, .setpoint = guidance::VelocitySetpoint{ setpoint } },
                .linear_quadratic_controller_input = control::LinearQuadraticControllerInput{ .Zt = Zt, .u_sol_trim = trim_sol.operating_point.input, .surface_actuators = surface_actuators, .propulsor_actuators = propulsor_actuators, .A = lin_sol.A, .B = lin_sol.B, .setpoint = guidance::LinearQuadraticSetpoint{ setpoint } },
                .nonlinear_controller_input = control::NonlinearControllerInput{ .Zt = Zt, .surface_actuators = surface_actuators, .propulsor_actuators = propulsor_actuators, .setpoint = guidance::NonlinearSetpoint{ setpoint } }
            };
            u_cmd = control_properties.step(controller_inputs, options.trim_bool);
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
        aerodynamics::AerodynamicWrench WB_aero = aerodynamics::step_aero_forces_moments(aerodynamic_properties, structural_properties, Xt, static_atm_state_t, u_surface_actual, windB);
        dynamics::Force FB_aero = WB_aero.F;
        dynamics::Moment MB_aero = WB_aero.M;

        // compute propulsive forces and momments
        propulsion::PropulsiveWrench WB_propulsive = propulsion::step_propulsive_forces_moments(propulsor_actuators, Xt, static_atm_state_t, u_propulsor_actual);
        dynamics::Force FB_propulsive = WB_propulsive.F;
        dynamics::Moment MB_propulsive = WB_propulsive.M;

        // compute gravitational force
        Eigen::Vector3d FB_g = mass.data * aircraft.FRDFrameNED.gB.data;

        // compute net forces and moments
        FB_net = dynamics::Force{ FB_g + FB_aero.data + FB_propulsive.data };
        MB_net = dynamics::Moment{ MB_aero.data + MB_propulsive.data };
        WB_net = dynamics::Wrench{ .F = FB_net, .M = MB_net };

        // compute rigid-body dynamics
        Xt = dynamics::step_rigid_body(Xt, mass, J, WB_net);

        // declare StepOptions
        vehicles::StepOptions StepOpts;

        // set step options
        StepOpts.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions{ .rbs_BN = Xt };
        aerodynamics::AerodynamicState aero_state_t = aerodynamics::compute_aerodynamic_state(Xt, windB);
        StepOpts.STABFrameFRDStepOpts = vehicles::STABFrameFRDStepOptions{ .ads = aero_state_t };
        StepOpts.WINDFrameSTABStepOpts = vehicles::WINDFrameSTABStepOptions{ .ads = aero_state_t };

        // step frames
        aircraft.step(StepOpts);

        // update data context
        io::DataContext data_context{
            .Xt=Xt,
            .Yt=Yt,
            .Zt=Zt,
            .u_surface=u_surface_actual,
            .u_propulsor=u_propulsor_actual,
            .WB_net=WB_net,
            .WB_aero=WB_aero,
            .WB_propulsive=WB_propulsive,
            .setpoint=setpoint,
            .windB=windB
        };

        // step data manager
        data_manager.step(t, data_context);

        // generate in_pkt from the simulation state
        messages::FlightGearMessageIn in_pkt = messages::process_in_pkt(
            geography::compute_geographic_state(aircraft.FRDFrameECEF),
            aircraft.FRDFrameNED.eulNB
        );

        // send packet
        udp_in.send(in_pkt);

        // print state
        if (options.verbose_bool) { aircraft.print_state(t, windB); }

        // step timer by dt
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(constants::dt));

        // sleep to maintain frequency dictated by dt
        std::this_thread::sleep_until(next);
    }
}
