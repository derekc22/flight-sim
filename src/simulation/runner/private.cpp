#include <chrono>
#include <string>
#include <thread>
#include <iomanip>
#include <sstream>
#include <spdlog/spdlog.h>
#include "simulation/runner/public.hpp"
#include "simulation/runner/private.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/avionics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/control/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/util/trig/public.hpp"
#include "simulation/estimation/public.hpp"
#include "simulation/frames/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/integrators/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/structural/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/vehicles/public.hpp"
#include "simulation/failures/public.hpp"
#include "simulation/settings/public.hpp"
#include "simulation/fsm/public.hpp"
#include "core/connection/public.hpp"
#include "core/devices/public.hpp"
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

    vehicles::Aircraft load_vehicle(const std::string& aircraft_id, const JSONFlags& json_flags) {
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
        aircraft.step(json::parse_initialization_config(json_flags.trim_flag));

        // parse settings
        settings::SettingsManager settings_manager = json::parse_settings_config(actuator_properties);

        // apply settings
        aircraft = settings_manager.populate(aircraft);

        return aircraft;
    }

    Scheduler::Scheduler(const ModuleRates& module_rates, int tf) : module_rates(module_rates) {
        double frac_guidance_steps = module_rates.guidance_hz / constants::hz; // fraction of steps that will call guidance
        guidance_tf = 1 + // for the immediate call at t=0
            std::floor((tf - 1) * frac_guidance_steps); // number of remaining steps that will call guidance
    }

    void Scheduler::step(fsm::FiniteState current_mode) {
        sensor_tick += module_rates.sensor_hz;
        ++sensor_elapsed_ticks;

        avionics_tick += module_rates.avionics_hz;
        ++avionics_elapsed_ticks;

        estimation_tick += module_rates.estimation_hz;
        ++estimation_elapsed_ticks;

        if (current_mode == fsm::FiniteState::Autopilot) {
            guidance_tick += module_rates.guidance_hz;
            control_tick += module_rates.control_hz;
            ++control_elapsed_ticks;
        }

        log_tick += module_rates.log_hz;
    }

    RunManager::RunManager(const CLIOptions& cli_options, const JSONOptions& json_options) : 
        cli_options(cli_options), 
        json_options(json_options),

        // load vehicle
        aircraft(load_vehicle(cli_options.aircraft_id, json_options.flags)),

        // initialize udp connections
        udp_out(5510),
        udp_in("127.0.0.1", 5511),

        // initialize scheduler
        scheduler(json_options.module_rates, json_options.tf),

        // start timer
        next(std::chrono::steady_clock::now()),

        // create state machine
        state_machine(json_options.flags)
    {
        // create data manager
        if (cli_options.flags.data_flag) {
            data_manager.emplace(json_options.tf, json_options.flags);
        }
        if (json_options.flags.rerun_flag) {
            rerun_manager.emplace(json_options.flags, json_options.module_rates.log_hz);
        }

        // create analysis manager
        if (cli_options.flags.analysis_flag) {
            // initialize analysis context
            analysis_manager.emplace(cli_options.aircraft_id, json_options.flags, json_options.module_rates);
        }

        // create joystick manager
        if (json_options.flags.joystick_flag) {
            actuators::ActuatorLimits actuator_limits = actuators::pack_actuator_limits(
                aircraft.actuator_properties.surface_actuators,
                aircraft.actuator_properties.propulsor_actuators
            );
            joystick_manager.emplace(actuator_limits);
        }
    }

    RunManager::~RunManager() = default;

    void RunManager::cleanup() {
        std::string data_dir_path = cli_options.data_dir_path;
        std::string log_dir_path = cli_options.log_dir_path;
        std::string report_dir_path = cli_options.report_dir_path;

        // dump configs
        json::dump_run_configs(log_dir_path);

        // save data
        if (data_manager) {
            data_manager->save(data_dir_path);
        }

        // save analysis data
        if (analysis_manager) {
            analysis_manager->save(data_dir_path, report_dir_path);
            json::dump_analyze_configs(log_dir_path);
        }
    }

    void RunManager::run() {
        for (int t = 0; t < json_options.tf; ++t) {
            step(t);
        }
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

        // get flags
        CLIFlags& cli_flags = cli_options.flags;
        JSONFlags& json_flags = json_options.flags;

        // fetch from FlightGear
        if (auto out_msg = udp_out.try_receive()) {
            cached_msg_out = messages::process_out_msg(out_msg.value());
        }

        // apply wind
        atmospheric::Wind windI { constants::Zero3 };
        atmospheric::Wind windB { constants::Zero3 };
        if (json_flags.wind_flag) {
            windI.data = cached_msg_out.wind.data;
            windB.data = frames::transform_vec(
                windI.data,
                aircraft.NEDFrameECEF,
                aircraft.FRDFrameNED
            );
        }

        // extract reusable quantities
        dynamics::Mass& mass = structural_properties.mass;

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

        // build autodiff model
        autodiff::AutoDiffModel autodiff_model = autodiff::build_autodiff_model(aircraft);

        // trim and linearization
        if (json_flags.trim_flag && !trim_sol.attempted) {
            trim_sol = trim::inspect_trim(aircraft, autodiff_model, windB);

            // update analysis context
            if (analysis_manager) { analysis_manager->context.trim_sol = trim_sol; }

            if (trim_sol.converged) {
                // obtain full state from trim solution
                dynamics::RigidBodyState Xt_trim = trim::update_state_from_trim(Xt, trim_sol.operating_point.state);
                aerodynamics::AerodynamicState aero_t_trim = aerodynamics::compute_aerodynamic_state(Xt_trim, trim_sol.conditions.windB);

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

                /** @deprecated */
                // overwrite actuator lag state with trim controls
                // trim::update_actuators_lag_from_trim(surface_actuators, propulsor_actuators, trim_sol);

                // linearize
                lin_sol = linearization::linearize_operating_point(
                    autodiff_model,
                    trim_sol.operating_point,
                    trim_sol.conditions
                );

                // perform eigenanalysis
                analysis::EigenAnalysis eig_sol = analysis::linearization_eigen_analysis(lin_sol);

                // update analysis context
                if (analysis_manager) {
                    analysis_manager->context.lin_sol = lin_sol;
                    analysis_manager->context.eig_sol = eig_sol;
                }
            }
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

        if (json_flags.avionics_flag) {
            if (scheduler.sensor_tick >= constants::hz) {
                double sensor_dt = scheduler.sensor_elapsed_ticks * constants::dt;

                // step sensors
                sensor_meas = sensor_properties.step(sensor_gt, sensor_dt);
                sensor_meas_t_1 = sensor_meas;

                scheduler.sensor_tick -= constants::hz;
                scheduler.sensor_elapsed_ticks = 0;
            }
            else sensor_meas = sensor_meas_t_1; // perform ZOH
        }

        if (json_flags.avionics_flag) {
            if (scheduler.avionics_tick >= constants::hz) {
                double avionics_dt = scheduler.avionics_elapsed_ticks * constants::dt;

                // aggregate ground truth avionics data
                avionics::AvionicsGroundTruth avionics_gt = avionics::build_avionics_gt(
                    Xt,
                    XEt,
                    aero_t,
                    atm_t,
                    geo_t
                );

                // step avionics
                avionics::AvionicsMeasurements avionics_meas = avionics_properties.step(sensor_meas, sensor_properties.hist, sensor_gt, avionics_gt, avionics_dt);

                // overwrite local measurement state with sensor measurements
                Yt = avionics::get_state_from_avionics(sensor_meas, avionics_meas, avionics_properties.settings);
                Yt_1 = Yt;

                scheduler.avionics_tick -= constants::hz;
                scheduler.avionics_elapsed_ticks = 0;
            }
            else Yt = Yt_1; // perform ZOH
        }

        // initialize estimated state to measurements
        dynamics::RigidBodyState Zt = Yt;

        if (json_flags.estimation_flag) {
            if (scheduler.estimation_tick >= constants::hz) {
                double estimation_dt = scheduler.estimation_elapsed_ticks * constants::dt;
                linearization::LocalLinearization estimator_lin_sol;
                operating::OperatingPoint estimator_operating_point;
                operating::OperatingConditions estimator_conditions;

                if (estimation_properties.extended_kalman_estimator_type == estimation::EstimatorType::ExtendedKalmanFilter) {
                    estimator_conditions = {
                        .atm = atm_t,
                        .windB = windB,
                        .steady_state = false
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
                        .u_cmd_t_1 = u_cmd_t_1,
                    },
                    .extended_kalman_estimator_input = estimation::ExtendedKalmanEstimatorInput {
                        .Yt = Yt,
                        .u_cmd_t_1 = u_cmd_t_1,
                        .model = autodiff_model,
                        .conditions = estimator_conditions
                    }
                };

                // overwrite local estimated state with estimator result
                Zt = estimation_properties.step(estimator_inputs, estimation_dt, json_flags.trim_flag).Zt;
                Zt_1 = Zt;

                scheduler.estimation_tick -= constants::hz;
                scheduler.estimation_elapsed_ticks = 0;
            }
            else Zt = Zt_1; // perform ZOH
        }

        // initialize guidance setpoint
        guidance::GuidanceSetpoint setpoint{};

        // initialize control commands
        control::ControlOutput u_cmd{};

        // declare for state machine
        bool mode_toggled = false;
        devices::JoystickOutput joystick_output;

        // fetch from joystick
        if (joystick_manager) {
            joystick_output = joystick_manager->step(u_cmd_t_1);
            mode_toggled = joystick_output.mode_toggled;
        }

        // step state machine
        fsm::FiniteState current_mode = state_machine.step(mode_toggled);

        if (current_mode == fsm::FiniteState::Manual) {
            u_cmd = joystick_output.u_cmd;
        }

        // no need to rate-limit as the trim command is fixed
        else if (current_mode == fsm::FiniteState::AutopilotTrim) {
            u_cmd = trim::set_control_inputs_from_trim(trim_sol.operating_point.input);
        }

        else if (current_mode == fsm::FiniteState::Autopilot) {
            if (scheduler.guidance_tick >= constants::hz) {
                setpoint = guidance_properties.step(scheduler.guidance_tf);
                setpoint_t_1 = setpoint;

                scheduler.guidance_tick -= constants::hz;
            }
            else setpoint = setpoint_t_1; // perform ZOH

            if (scheduler.control_tick >= constants::hz) {
                double control_dt = scheduler.control_elapsed_ticks * constants::dt;
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
                u_cmd = control_properties.step(controller_inputs, control_dt, json_flags.trim_flag);

                scheduler.control_tick -= constants::hz;
                scheduler.control_elapsed_ticks = 0;
            }
            else u_cmd = u_cmd_t_1; // perform ZOH
        }

        // apply fixed actuator inputs
        actuators::FixedActuatorInputs fixed_inputs = actuator_properties.settings.get_fixed_actuator_inputs();
        u_cmd.surface_inputs.flap_cmd = fixed_inputs.flap;
        u_cmd.surface_inputs.spoiler_cmd = fixed_inputs.spoiler;

        // update prior-step control command
        u_cmd_t_1 = u_cmd;

        actuators::ActuatorInputs_T<double> u_actual{};

        // apply surface actuator dynamics
        u_actual.surface_inputs = actuator_properties.step(u_cmd.surface_inputs, constants::dt);

        // apply propulsor actuator dynamics
        u_actual.propulsor_inputs = actuator_properties.step(u_cmd.propulsor_inputs, constants::dt);

        integrators::RK4Model rk4_model{
            .structural = structural_properties,
            .aerodynamic = aerodynamic_properties,
            .propulsor_actuators = propulsor_actuators
        };

        operating::OperatingConditions rk4_conditions{
            .atm = atm_t,
            .windI = windI,
            .steady_state = false
        };

        // compute forces, moments, and next-step rigid body state
        integrators::RK4Output rk4_out = integrators::step_rigid_body_rk4(
            Xt,
            rk4_model,
            rk4_conditions,
            u_actual,
            constants::dt
        );
        dynamics::RigidBodyState Xt1 = rk4_out.Xt1;
        dynamics::Wrench WB_net = rk4_out.WB_set.net;

        // update data context
        io::DataContext data_context{
            .Xt=Xt,
            .Yt=Yt,
            .Zt=Zt,
            .u_surface=u_actual.surface_inputs,
            .u_propulsor=u_actual.propulsor_inputs,
            .u_surface_commanded=u_cmd.surface_inputs,
            .u_propulsor_commanded=u_cmd.propulsor_inputs,
            .WB_net=WB_net,
            .WB_aerodynamic=rk4_out.WB_set.aerodynamic,
            .WB_propulsive=rk4_out.WB_set.propulsive,
            .setpoint=setpoint,
            .windB=windB
        };

        // step data manager
        if (data_manager) {
            data_manager->step(t, data_context);
        }

        if (scheduler.log_tick >= constants::hz) {
            // step rerun manager
            if (rerun_manager) { 
                rerun_manager->step(t, data_context); 
            }

            // log state
            if (json_flags.verbose_flag) {
                log_state(t, Xt, geo_t, aero_t, windI);
            }
            scheduler.log_tick -= constants::hz;
        }

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

        // step scheduler
        scheduler.step(current_mode);

        // step timer by dt
        next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(constants::dt)
        );

        // sleep to maintain frequency dictated by dt
        if (!cli_flags.fast_flag) {
            std::this_thread::sleep_until(next);
        }
    }

    std::string print_vec(const char* name, const Eigen::Vector3d& x, const char* unit) {
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);

        ss
            << std::left << std::setw(8) << name
            << "[ "
            << std::right << std::setw(10) << x.x() << ", "
            << std::right << std::setw(10) << x.y() << ", "
            << std::right << std::setw(10) << x.z()
            << " ] " << unit << '\n';

        return ss.str();
    }

    void log_state(
        int t,
        const dynamics::RigidBodyState& Xt,
        const geography::GeographicState& geo,
        const aerodynamics::AerodynamicState& aero,
        const atmospheric::Wind& windI
    ) {
        const Eigen::Vector3d& p = Xt.p.data;

        dynamics::EulerAngles eul;
        eul.set(Xt.q);

        const Eigen::Vector3d& v = Xt.v.data;
        const Eigen::Vector3d& w = Xt.w.data;
        const Eigen::Vector3d& g = geography::gB(Xt.q).data;
        const Eigen::Vector3d& wind = windI.data;

        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);

        ss
            << "\n"
            << "t       " << t * constants::dt << " [s]\n"
            << "---------------------------------------------------------------------------------\n";

        ss << print_vec("p", p, "[m]");

        ss
            << std::left << std::setw(8) << "eul"
            << "[ "
            << std::right << std::setw(10) << util::rad_to_deg(eul.psi()) << ", "
            << std::right << std::setw(10) << util::rad_to_deg(eul.theta()) << ", "
            << std::right << std::setw(10) << util::rad_to_deg(eul.phi())
            << " ] [deg]\n";

        ss << print_vec("v", v, "[m/s]");

        ss
            << std::left << std::setw(8) << "w"
            << "[ "
            << std::right << std::setw(10) << util::rad_to_deg(w.x()) << ", "
            << std::right << std::setw(10) << util::rad_to_deg(w.y()) << ", "
            << std::right << std::setw(10) << util::rad_to_deg(w.z())
            << " ] [deg/s]\n";

        ss << print_vec("g", g, "[m/s^2]");
        ss << print_vec("wind", wind, "[m/s]");

        ss
            << std::left << std::setw(8) << "geo"
            << "lat: " << std::right << std::setw(10) << util::rad_to_deg(geo.lat.data) << " [deg], "
            << "lon: " << std::right << std::setw(10) << util::rad_to_deg(geo.lon.data) << " [deg], "
            << "alt: " << std::right << std::setw(10) << geo.alt.data << " [m]\n";

        ss
            << std::left << std::setw(8) << "aero"
            << "alpha: " << std::right << std::setw(10) << util::rad_to_deg(aero.alpha.data) << " [deg], "
            << "beta: "  << std::right << std::setw(10) << util::rad_to_deg(aero.beta.data)  << " [deg]\n";

        ss
            << "---------------------------------------------------------------------------------\n";

        spdlog::info(ss.str());
    }

}
