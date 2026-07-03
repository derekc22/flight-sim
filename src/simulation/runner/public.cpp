#include <chrono>
#include <string>
#include <thread>
#include "simulation/runner/public.hpp"
#include "simulation/runner/private.hpp"
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
#include "simulation/failures/public.hpp"
#include "simulation/settings/public.hpp"
#include "core/connection/public.hpp"
#include "core/json/actuators/public.hpp"
#include "core/json/aerodynamics/public.hpp"
#include "core/json/avionics/public.hpp"
#include "core/json/sensors/public.hpp"
#include "core/json/control/public.hpp"
#include "core/json/estimation/public.hpp"
#include "core/json/guidance/public.hpp"
#include "core/json/initialization/public.hpp"
#include "core/json/settings/public.hpp"
#include "core/json/public.hpp"
#include "core/json/structural/public.hpp"
#include "core/messages/public.hpp"
#include "analysis/eigenanalysis/public.hpp"

namespace runner {

    vehicles::Aircraft load_vehicle(const std::string& aircraft_id, bool trim_flag) {
        structural::StructuralProperties structural_properties = json::parse_structural_config();
        actuators::ActuatorProperties actuator_properties = json::parse_actuator_config(structural_properties);
        control::ControlProperties control_properties = json::parse_control_config();

        // create vehicle from config
        vehicles::Aircraft aircraft {
            aircraft_id,
            structural_properties,
            json::parse_aerodynamics_config(structural_properties),
            actuator_properties,
            control_properties,
            json::parse_sensors_config(),
            json::parse_avionics_config(),
            json::parse_guidance_config(control_properties),
            json::parse_estimation_config()
        };

        // set initial conditions from config
        aircraft.step(json::parse_initialization_config(trim_flag));

        // parse settings
        settings::SettingsManager settings_manager = json::parse_settings_config(actuator_properties);

        // apply settings
        aircraft = settings_manager.populate(aircraft);

        return aircraft;
    }

    void MultiRateAccumulator::step(const JSONOptions& json_options) {
        sensor_acc += json_options.sensor_hz;
        avionics_acc += json_options.avionics_hz;
        estimation_acc += json_options.estimation_hz;
        guidance_acc += json_options.guidance_hz;
        control_acc += json_options.control_hz;
        log_acc += json_options.log_hz;
    }

    RunManager::RunManager(CLIOptions cli_options, JSONOptions json_options) :
        cli_options(cli_options),
        json_options(json_options),

        // load vehicle
        aircraft(load_vehicle(cli_options.aircraft_id, json_options.trim_flag)),

        // create data manager
        data_manager(json_options.tf, cli_options.data_flag, json_options.control_flag, json_options.avionics_flag, json_options.estimation_flag, json_options.wind_flag),
        rerun_manager(json_options.rerun_flag, json_options.control_flag, json_options.avionics_flag, json_options.estimation_flag, json_options.wind_flag, json_options.log_hz),
        
        // create analysis manager
        analysis_manager{.data_flag=cli_options.data_flag, .analysis_flag=cli_options.analysis_flag, .trim_flag=json_options.trim_flag},

        // initialize udp connections
        udp_out(5510),
        udp_in("127.0.0.1", 5511),

        // start timer
        next(std::chrono::steady_clock::now()) {}

    RunManager::~RunManager() = default;

    void RunManager::cleanup() {
        std::string data_dir_path = cli_options.data_dir_path;
        std::string log_dir_path = cli_options.log_dir_path;
        std::string report_dir_path = cli_options.report_dir_path;

        // save data
        data_manager.save(data_dir_path);
        analysis_manager.save(data_dir_path, report_dir_path);

        // dump configs
        json::dump_configs(log_dir_path);
    }

    void RunManager::run() {
        for (int t = 0; t < json_options.tf; ++t) {
            step(t);
        }

        // cleanup
        cleanup();
    }

    void RunManager::step(int t) {
        // get aircraft properties
        structural::StructuralProperties& structural_properties = aircraft.structural_properties;
        aerodynamics::AerodynamicProperties& aerodynamic_properties = aircraft.aerodynamic_properties;
        control::ControlProperties& control_properties = aircraft.control_properties;
        estimation::EstimationProperties& estimation_properties = aircraft.estimation_properties;
        actuators::ActuatorProperties& actuator_properties = aircraft.actuator_properties;
        guidance::GuidanceProperties& guidance_properties = aircraft.guidance_properties;
        sensors::SensorProperties& sensor_properties = aircraft.sensor_properties;
        avionics::AvionicsProperties& avionics_properties = aircraft.avionics_properties;
        JSONOptions& options = json_options; 

        // fetch from FlightGear
        if (auto out_msg = udp_out.try_receive()) {
            cached_msg_out = messages::process_out_msg(out_msg.value());
        }

        // apply wind
        atmospheric::Wind windB { constants::Zero3 };
        if (options.wind_flag) {
            windB.data = frames::transform_vec(
                cached_msg_out.wind.data,
                aircraft.NEDFrameECEF,
                aircraft.FRDFrameNED
            );
        }

        // extract reuseable quantities
        dynamics::Mass mass = structural_properties.mass;
        dynamics::InertiaTensor JB = structural_properties.JB;

        actuators::SurfaceActuators& surface_actuators = actuator_properties.surface_actuators;
        actuators::PropulsorActuators& propulsor_actuators = actuator_properties.propulsor_actuators;

        // compute rigid body states
        dynamics::RigidBodyState Xt = dynamics::compute_rigid_body_state(aircraft.FRDFrameNED);
        dynamics::RigidBodyState XEt = dynamics::compute_rigid_body_state(aircraft.FRDFrameECEF);

        // compute aerodynamic state
        aerodynamics::AerodynamicState aero_t = aerodynamics::compute_aerodynamic_state(Xt, windB);

        // compute geographic state
        geography::GeographicState geo_t = geography::compute_geographic_state(aircraft.FRDFrameECEF);

        // compute static atmospheric state
        atmospheric::StaticAtmosphericState atm_t = atmospheric::compute_static_atmospheric_state(aircraft.FRDFrameECEF);

        // trim and linearization
        if (options.trim_flag && !trim_sol.attempted) {
            trim_sol = trim::inspect_trim(aircraft, windB);

            // initialize analysis context
            io::AnalysisContext analysis_context{
                .aircraft_id=aircraft.id
            };

            // update analysis context
            analysis_context.trim_sol = trim_sol;

            if (trim_sol.converged) {
                // obtain full state from trim solution
                auto [Xt_trim, aero_t_trim] = trim::update_state_from_trim(Xt, trim_sol);

                dynamics::Wrench WB_net_trim = trim_sol.wrench;

                // define TrimStepOptions
                vehicles::StepOptions TrimStepOptions;

                // overwrite state with trim state
                TrimStepOptions.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions{ .X_BN = Xt_trim };
                TrimStepOptions.STABFrameFRDStepOpts = vehicles::STABFrameFRDStepOptions{ .aero = aero_t_trim };
                TrimStepOptions.WINDFrameSTABStepOpts = vehicles::WINDFrameSTABStepOptions{ .aero = aero_t_trim };

                // step frames
                aircraft.step(TrimStepOptions);

                // overwrite local state with trim state
                Xt = Xt_trim;
                XEt = dynamics::compute_rigid_body_state(aircraft.FRDFrameECEF);
                aero_t = aerodynamics::compute_aerodynamic_state(Xt, windB);
                geo_t = geography::compute_geographic_state(aircraft.FRDFrameECEF);
                atm_t = atmospheric::compute_static_atmospheric_state(aircraft.FRDFrameECEF);

                // overwrite internal state with trim state
                WB_net_t_1 = WB_net_trim;
                auto [u_surface_trim, u_propulsor_trim] = trim::update_actuators_from_trim(
                    u_surface_actual_prev,
                    u_propulsor_actual_prev,
                    trim_sol
                );
                u_surface_actual_prev = u_surface_trim;
                u_propulsor_actual_prev = u_propulsor_trim;

                /** @deprecated */
                // overwrite actuator lag state with trim controls
                // trim::update_actuators_lag_from_trim(surface_actuators, propulsor_actuators, trim_sol);

                // linearize
                lin_sol = linearization::linearize_operating_point(
                    aircraft,
                    trim_sol.operating_point,
                    trim_sol.conditions
                );

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

        // aggregate ground truth sensor data
        sensors::SensorGroundTruth sensor_gt = sensors::build_sensor_gt(
            Xt,
            XEt,
            aero_t,
            atm_t,
            mass,
            WB_net_t_1
        );

        // use sensors and avionics
        sensors::SensorMeasurements sensor_meas;

        if (options.avionics_flag) {
            if (acc.sensor_acc >= constants::hz) {

                // step sensors
                sensor_meas = sensor_properties.step(sensor_gt);
                sensor_meas_t_1 = sensor_meas;

                acc.sensor_acc -= constants::hz;
            }
            else sensor_meas = sensor_meas_t_1; // perform ZOH
        }

        if (options.avionics_flag) {
            if (acc.avionics_acc >= constants::hz) {
                // aggregate ground truth avionics data
                avionics::AvionicsGroundTruth avionics_gt = avionics::build_avionics_gt(
                    Xt,
                    XEt,
                    aero_t,
                    atm_t,
                    geo_t
                );

                // step avionics
                avionics::AvionicsMeasurements avionics_meas = avionics_properties.step(
                    sensor_meas, 
                    sensor_properties.hist,
                    sensor_gt,
                    avionics_gt
                );

                // overwrite local measurement state with sensor measurements
                Yt = avionics::get_state_from_avionics(sensor_meas, avionics_meas, avionics_properties.settings);
                Yt_1 = Yt;

                acc.avionics_acc -= constants::hz;
            }
            else Yt = Yt_1; // perform ZOH
        }

        // initialize estimated state to measurements
        dynamics::RigidBodyState Zt = Yt;

        if (options.estimation_flag) {
            if (acc.estimation_acc >= constants::hz) {
                linearization::LocalLinearization estimator_lin_sol;
                operating::OperatingPoint estimator_operating_point;
                operating::OperatingConditions estimator_conditions;

                if (estimation_properties.extended_kalman_estimator_type == estimation::EstimatorType::ExtendedKalmanFilter) {
                    dynamics::State_T<double> yt = dynamics::pack_state(Yt);
                    actuators::ActuatorInputs_T<double> actuator_inputs = actuators::pack_actuator_inputs(
                        u_surface_actual_prev,
                        u_propulsor_actual_prev
                    );

                    estimator_operating_point = operating::OperatingPoint {
                        .state = yt,
                        .input = actuator_inputs
                    };

                    estimator_conditions = {
                        .atm = atm_t,
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

                // overwrite local estimated state with estimator result
                Zt = estimation_properties.step(estimator_inputs, options.trim_flag).Zt;
                Zt_1 = Zt;

                acc.estimation_acc -= constants::hz;
            }
            else Zt = Zt_1; // peform ZOH
        }

        // specify guidance setpoint
        guidance::GuidanceSetpoint setpoint{};
        if (options.control_flag) {
            if (acc.guidance_acc >= constants::hz) {
                setpoint = guidance_properties.step(options.tf);
                setpoint_t_1 = setpoint;

                acc.guidance_acc -= constants::hz;
            }
            else setpoint = setpoint_t_1; // perform ZOH
        }

        // specify control commands
        control::ControlOutput u_cmd{};

        if (options.trim_flag && !options.control_flag) {
            u_cmd = trim::set_control_inputs_from_trim(trim_sol);
        }

        if (options.control_flag) {
            if (acc.control_acc >= constants::hz) {
                control::ControllerInputs controller_inputs {
                    .attitude_controller_input = control::AttitudeControllerInput{
                        .Zt = Zt,
                        .surface_actuators = surface_actuators,
                        .setpoint = guidance::AttitudeSetpoint{ setpoint }
                    },
                    .velocity_controller_input = control::VelocityControllerInput{
                        .Zt = Zt,
                        .propulsor_actuators = propulsor_actuators,
                        .setpoint = guidance::VelocitySetpoint{ setpoint }
                    },
                    .linear_quadratic_controller_input = control::LinearQuadraticControllerInput{
                        .Zt = Zt,
                        .u_sol_trim = trim_sol.operating_point.input,
                        .surface_actuators = surface_actuators,
                        .propulsor_actuators = propulsor_actuators,
                        .A = lin_sol.A,
                        .B = lin_sol.B,
                        .setpoint = guidance::LinearQuadraticSetpoint{ setpoint }
                    },
                    .nonlinear_controller_input = control::NonlinearControllerInput{
                        .Zt = Zt,
                        .surface_actuators = surface_actuators,
                        .propulsor_actuators = propulsor_actuators,
                        .setpoint = guidance::NonlinearSetpoint{ setpoint }
                    }
                };
                u_cmd = control_properties.step(controller_inputs, options.trim_flag);
                u_cmd_t_1 = u_cmd;

                acc.control_acc -= constants::hz;
            }
            else u_cmd = u_cmd_t_1; // perform ZOH
        }

        // apply fixed actuator inputs
        actuators::FixedActuatorInputs fixed_inputs = actuator_properties.settings.get_fixed_actuator_inputs();
        u_cmd.surface_inputs.flap_cmd = fixed_inputs.flap;
        u_cmd.surface_inputs.spoiler_cmd = fixed_inputs.spoiler;

        // apply surface actuator dynamics
        actuators::SurfaceActuatorInputs_T<double> u_surface_actual = actuator_properties.step(u_cmd.surface_inputs);

        // apply propulsor dynamics
        actuators::PropulsorActuatorInputs_T<double> u_propulsor_actual = actuator_properties.step(u_cmd.propulsor_inputs);
        u_surface_actual_prev = u_surface_actual;
        u_propulsor_actual_prev = u_propulsor_actual;

        // compute aerodynamic forces and moments
        aerodynamics::AerodynamicWrench WB_aero = aerodynamics::step_aero_forces_moments(
            aerodynamic_properties,
            Xt,
            atm_t,
            u_surface_actual,
            windB
        );
        dynamics::Force FB_aero = WB_aero.F;
        dynamics::Moment MB_aero = WB_aero.M;

        // compute propulsive forces and momments
        propulsion::PropulsiveWrench WB_propulsive = propulsion::step_propulsive_forces_moments(
            propulsor_actuators,
            Xt,
            atm_t,
            u_propulsor_actual
        );
        dynamics::Force FB_propulsive = WB_propulsive.F;
        dynamics::Moment MB_propulsive = WB_propulsive.M;

        // compute gravitational force
        Eigen::Vector3d FB_g = mass.data * aircraft.FRDFrameNED.gB.data;

        // compute net forces and moments
        dynamics::Force FB_net{ FB_g + FB_aero.data + FB_propulsive.data };
        dynamics::Moment MB_net{ MB_aero.data + MB_propulsive.data };
        dynamics::Wrench WB_net{ .F = FB_net, .M = MB_net };

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

        if (acc.log_acc >= constants::hz) {
            // step rerun manager
            rerun_manager.step(t, data_context);

            // log state
            if (options.verbose_flag) {
                log_state(t, Xt, geo_t, aero_t, windB);
            }
            acc.log_acc -= constants::hz;
        }

        // compute next-step rigid body state
        dynamics::RigidBodyState Xt1{ dynamics::step_rigid_body(Xt, mass, JB, WB_net) };

        // compute next-step aerodynamic state
        aerodynamics::AerodynamicState aero_t1 = aerodynamics::compute_aerodynamic_state(Xt1, windB);

        // set step options
        vehicles::StepOptions StepOpts;
        StepOpts.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions{ .X_BN = Xt1 };
        StepOpts.STABFrameFRDStepOpts = vehicles::STABFrameFRDStepOptions{ .aero = aero_t1 };
        StepOpts.WINDFrameSTABStepOpts = vehicles::WINDFrameSTABStepOptions{ .aero = aero_t1 };

        // step frames
        aircraft.step(StepOpts);

        // advance internal state
        WB_net_t_1 = WB_net;

        // compute next-step geographic state
        geography::GeographicState geo_t1 = geography::compute_geographic_state(aircraft.FRDFrameECEF);

        // check for runtime failures
        geography::HeightAGL height_agl{ geo_t1.alt.data - cached_msg_out.ground_elevation.data };
        failures::FailureInputs failure_inputs {
            .height_agl = height_agl
        };
        failure_manager.check_runtime_failures(failure_inputs);

        // generate in_msg from the simulation state
        messages::FlightGearMessageIn in_msg = messages::process_in_msg(
            geo_t1,
            aircraft.FRDFrameNED.eulNB
        );

        // send message
        udp_in.send(in_msg);

        // step accumulator
        acc.step(options);

        // step timer by dt
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(constants::dt)
        );

        // sleep to maintain frequency dictated by dt
        if (!cli_options.fast_flag) {
            std::this_thread::sleep_until(next);
        }
    }

}
