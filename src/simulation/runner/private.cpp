#include <chrono>
#include <string>
#include <thread>
#include <array>
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
#include "simulation/util/print/public.hpp"
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
#include "simulation/allocator/public.hpp"
#include "core/connection/public.hpp"
#include "core/devices/public.hpp"
#include "core/json/actuators/public.hpp"
#include "core/json/aerodynamics/public.hpp"
#include "core/json/allocator/public.hpp"
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
        control::ControlProperties control_properties = json::parse_control_config(json_flags.trim_flag);

        // create vehicle from config
        vehicles::Aircraft aircraft {
            aircraft_id,
            structural_properties,
            json::parse_aerodynamics_config(),
            actuator_properties,
            control_properties,
            json::parse_sensors_config(),
            json::parse_avionics_config(),
            json::parse_guidance_config(control_properties),
            json::parse_estimation_config(json_flags.trim_flag),
            json::parse_allocator_config()
        };

        // set initial conditions from config
        vehicles::StepOptions InitStepOpts = json::parse_initialization_config(json_flags.trim_flag);
        structural::StructuralState struc_t = aircraft.structural_properties.compute_structural_state();
        InitStepOpts.CGFrameFRDStepOpts = vehicles::CGFrameFRDStepOptions{ .pB_GB = dynamics::Position{ struc_t.pB_GB.data } };
        aircraft.step(InitStepOpts);

        // parse settings
        settings::SettingsManager settings_manager = json::parse_settings_config(actuator_properties);

        // apply settings
        aircraft = settings_manager.populate(aircraft);

        return aircraft;
    }

    Scheduler::Scheduler(const ModuleRates& module_rates, int tf)
        : module_rates(module_rates)
    {
        double frac_guidance_steps = module_rates.guidance_hz / constants::hz; // fraction of steps that will call guidance
        guidance_tf = 1 +  // for the immediate call at t=0
            static_cast<int>(
                std::floor((tf - 1) * frac_guidance_steps) // number of remaining steps that will call guidance
            );
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

        // create state machine
        state_machine(json_options.flags)
    {
        // set u_actual_t_1 to match actuators' neutral initialization
        u_actual_t_1 = actuators::get_neutral_actuator_inputs(
            aircraft.actuator_properties.surface_actuators,
            aircraft.actuator_properties.propulsor_actuators
        );

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

        // start timer
        next = std::chrono::steady_clock::now();
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
        StepContext context = prepare_step();
        initialize_trim(context);
        step_measurements(context);
        step_estimation(context);
        step_control(context);
        step_physics(context);
        publish_step(t, context);
        finish_step(context.current_mode);
    }

    StepContext RunManager::prepare_step() {
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

        // compute rigid body states
        dynamics::RigidBodyState Xt = dynamics::compute_rigid_body_state(aircraft.CGFrameFRD, aircraft.NEDFrameECEF);
        dynamics::RigidBodyState XEt = dynamics::compute_rigid_body_state(aircraft.CGFrameFRD, aircraft.ECEFFrame);

        // compute structural state
        structural::StructuralState struc_t = aircraft.structural_properties.compute_structural_state();

        // compute aerodynamic state
        aerodynamics::AerodynamicState aero_t = aerodynamics::compute_aerodynamic_state(aircraft.CGFrameFRD, aircraft.NEDFrameECEF, windB);

        // compute geographic state
        geography::GeographicState geo_t = geography::compute_geographic_state(aircraft.FRDFrameECEF, aircraft.ECEFFrame);

        // compute static atmospheric state
        atmospheric::StaticAtmosphericState atm_t = atmospheric::compute_static_atmospheric_state(aircraft.FRDFrameECEF, aircraft.ECEFFrame);

        // build autodiff model
        autodiff::AutoDiffModel autodiff_model = autodiff::build_autodiff_model(aircraft, struc_t);

        // initialize transient conditions
        operating::OperatingConditions transient_conditions{
            .atm = atm_t,
            .windB = windB,
            .steady_state = false
        };

        return StepContext {
            .windI = windI,
            .windB = windB,
            .Xt = Xt,
            .XEt = XEt,
            .struc_t = struc_t,
            .aero_t = aero_t,
            .geo_t = geo_t,
            .atm_t = atm_t,
            .autodiff_model = autodiff_model,
            .transient_conditions = transient_conditions
        };
    }

    void RunManager::initialize_trim(StepContext& context) {
        // get aircraft properties
        actuators::ActuatorProperties& actuator_properties = aircraft.actuator_properties;
        actuators::SurfaceActuators& surface_actuators = actuator_properties.surface_actuators;
        actuators::PropulsorActuators& propulsor_actuators = actuator_properties.propulsor_actuators;

        // get flags
        JSONFlags& json_flags = json_options.flags;

        // trim and linearization
        if (json_flags.trim_flag && !trim_sol.attempted) {
            trim_sol = trim::inspect_trim(aircraft, context.autodiff_model, context.windB);

            // update analysis context
            if (analysis_manager) { analysis_manager->context.trim_sol = trim_sol; }

            if (trim_sol.converged) {
                // obtain full state from trim solution
                dynamics::RigidBodyState Xt_trim = trim::update_state_from_trim(context.Xt, trim_sol.operating_point.state);
                aerodynamics::AerodynamicState aero_t_trim = aerodynamics::compute_aerodynamic_state(Xt_trim, trim_sol.conditions.windB);

                dynamics::Position pB_GB{ context.struc_t.pB_GB.data };
                dynamics::RigidBodyState X_BN_trim = aircraft.rebase_cg_state(Xt_trim, pB_GB);

                // define TrimStepOptions
                vehicles::StepOptions TrimStepOptions;

                // overwrite state with trim state
                TrimStepOptions.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions{ .X_BN = X_BN_trim };
                TrimStepOptions.CGFrameFRDStepOpts = vehicles::CGFrameFRDStepOptions{ .pB_GB = pB_GB };
                TrimStepOptions.STABFrameFRDStepOpts = vehicles::STABFrameFRDStepOptions{ .aero = aero_t_trim };
                TrimStepOptions.WINDFrameSTABStepOpts = vehicles::WINDFrameSTABStepOptions{ .aero = aero_t_trim };

                // step frames
                aircraft.step(TrimStepOptions);

                // overwrite local state with trim state
                context.Xt = Xt_trim;
                context.XEt = dynamics::compute_rigid_body_state(aircraft.CGFrameFRD, aircraft.ECEFFrame);
                context.aero_t = aerodynamics::compute_aerodynamic_state(context.Xt, context.windB);
                context.geo_t = geography::compute_geographic_state(aircraft.FRDFrameECEF, aircraft.ECEFFrame);
                context.atm_t = atmospheric::compute_static_atmospheric_state(aircraft.FRDFrameECEF, aircraft.ECEFFrame);

                dynamics::Wrench WB_net_trim = {
                    .F = dynamics::Force{ trim_sol.wrench.F },
                    .M = dynamics::Moment{ trim_sol.wrench.M },
                };

                // overwrite internal state with trim state
                WB_net_t_1 = WB_net_trim;

                // overwrite actuator lag state with trim controls
                trim::update_actuators_lag_from_trim(surface_actuators, propulsor_actuators, trim_sol);
                u_actual_t_1 = trim_sol.operating_point.input;

                // compute linearization
                lin_sol = linearization::linearize_operating_point(
                    context.autodiff_model,
                    trim_sol.operating_point,
                    trim_sol.conditions
                );

                // compute virtual linearization
                virtual_lin_sol = linearization::linearize_virtual_operating_point(
                    context.autodiff_model,
                    operating::VirtualOperatingPoint_T<double>{
                        .state=trim_sol.operating_point.state,
                        .input=trim_sol.wrench
                    }
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
    }

    void RunManager::step_measurements(StepContext& context) {
        // extract reusable quantities
        sensors::SensorProperties& sensor_properties = aircraft.sensor_properties;
        avionics::AvionicsProperties& avionics_properties = aircraft.avionics_properties;
        JSONFlags& json_flags = json_options.flags;

        // initialize measurements to ground truth
        context.Yt = context.Xt;

        // aggregate ground truth sensor data
        sensors::SensorGroundTruth sensor_gt = sensors::build_sensor_gt(
            context.Xt,
            context.XEt,
            context.aero_t,
            context.atm_t,
            context.struc_t.mass,
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
                    context.Xt,
                    context.XEt,
                    context.aero_t,
                    context.atm_t,
                    context.geo_t
                );

                // step avionics
                avionics::AvionicsMeasurements avionics_meas = avionics_properties.step(sensor_meas, sensor_properties.hist, sensor_gt, avionics_gt, avionics_dt);

                // overwrite local measurement state with sensor measurements
                context.Yt = avionics::get_state_from_avionics(sensor_meas, avionics_meas, avionics_properties.settings);
                Yt_1 = context.Yt;

                scheduler.avionics_tick -= constants::hz;
                scheduler.avionics_elapsed_ticks = 0;
            }
            else context.Yt = Yt_1; // perform ZOH
        }
    }

    void RunManager::step_estimation(StepContext& context) {
        estimation::EstimationProperties& estimation_properties = aircraft.estimation_properties;
        JSONFlags& json_flags = json_options.flags;

        // initialize estimated state to measurements
        context.Zt = context.Yt;

        if (json_flags.estimation_flag) {
            if (scheduler.estimation_tick >= constants::hz) {
                double estimation_dt = scheduler.estimation_elapsed_ticks * constants::dt;

                estimation::EstimatorInputs estimator_inputs = estimation_properties.build_estimator_inputs(
                    context.Yt,
                    trim_sol, lin_sol,
                    context.autodiff_model,
                    u_actual_t_1,
                    context.transient_conditions
                );

                // overwrite local estimated state with estimator result
                context.Zt = estimation_properties.step(estimator_inputs, estimation_dt).Zt;
                Zt_1 = context.Zt;

                scheduler.estimation_tick -= constants::hz;
                scheduler.estimation_elapsed_ticks = 0;
            }
            else context.Zt = Zt_1; // perform ZOH
        }
    }

    void RunManager::step_control(StepContext& context) {
        control::ControlProperties& control_properties = aircraft.control_properties;
        actuators::ActuatorProperties& actuator_properties = aircraft.actuator_properties;
        guidance::GuidanceProperties& guidance_properties = aircraft.guidance_properties;
        allocator::AllocatorProperties& allocator_properties = aircraft.allocator_properties;
        actuators::SurfaceActuators& surface_actuators = actuator_properties.surface_actuators;
        actuators::PropulsorActuators& propulsor_actuators = actuator_properties.propulsor_actuators;

        // initialize active mask
        std::array<bool, constants::virtual_input_dim> active_mask;
        std::array<bool, constants::input_dim> actuator_mask;

        // initialize guidance setpoint
        context.setpoint = {};

        // initialize virtual control command
        control::VirtualControlOutput mu_cmd{};

        // initialize control command
        context.u_cmd = {};

        // declare for state machine
        bool mode_toggled = false;
        devices::JoystickOutput joystick_output;

        // fetch from joystick
        if (joystick_manager) {
            joystick_output = joystick_manager->step(u_cmd_t_1);
            mode_toggled = joystick_output.mode_toggled;
        }

        // step state machine
        context.current_mode = state_machine.step(mode_toggled);

        if (context.current_mode == fsm::FiniteState::Manual) {
            context.u_cmd = joystick_output.u_cmd;
        }

        // no need to rate-limit as the trim command is fixed
        else if (context.current_mode == fsm::FiniteState::AutopilotTrim) {
            mu_cmd = {};
            util::fill_arr(active_mask, 0, 6, true);
            util::fill_arr(actuator_mask, 0, 6, true);
        }

        else if (context.current_mode == fsm::FiniteState::Autopilot) {
            if (scheduler.guidance_tick >= constants::hz) {
                context.setpoint = guidance_properties.step(scheduler.guidance_tf);
                setpoint_t_1 = context.setpoint;

                scheduler.guidance_tick -= constants::hz;
            }
            else context.setpoint = setpoint_t_1; // perform ZOH

            if (scheduler.control_tick >= constants::hz) {
                double control_dt = scheduler.control_elapsed_ticks * constants::dt;

                control::ControllerInputs controller_inputs = control_properties.build_controller_inputs(
                    context.Zt,
                    trim_sol, virtual_lin_sol,
                    surface_actuators, propulsor_actuators, 
                    context.setpoint,
                    delta_mu_vec_t_1
                );

                control::VirtualControlOutputSet virtual_ctrl_out = control_properties.step(controller_inputs, control_dt);
                mu_cmd = virtual_ctrl_out.mu;
                active_mask = virtual_ctrl_out.active_mask;
                actuator_mask = virtual_ctrl_out.actuator_mask;

                mu_cmd_t_1 = mu_cmd;
                active_mask_t_1 = active_mask;
                actuator_mask_t_1 = actuator_mask;

                scheduler.control_tick -= constants::hz;
                scheduler.control_elapsed_ticks = 0;
            }
            else {
                mu_cmd = mu_cmd_t_1; // perform ZOH
                active_mask = active_mask_t_1;
                actuator_mask = actuator_mask_t_1;
            }
        }

        // step control allocator
        if (context.current_mode == fsm::FiniteState::AutopilotTrim || context.current_mode == fsm::FiniteState::Autopilot) {
            control::ControlOutputSet ctrl_out = allocator_properties.step(
                allocator::build_allocator_input(
                    mu_cmd,
                    active_mask,
                    actuator_mask,
                    context.Zt, u_actual_t_1,
                    trim_sol.converged ? std::make_optional(trim_sol.operating_point.input) : std::nullopt,
                    context.transient_conditions,
                    context.autodiff_model
                )
            );
            context.u_cmd = ctrl_out.u;
            delta_mu_vec_t_1 = ctrl_out.delta_mu_vec_t_1;
        }

        // apply fixed actuator inputs
        actuators::FixedActuatorInputs fixed_inputs = actuator_properties.settings.get_fixed_actuator_inputs();
        context.u_cmd.surface_inputs.flap_cmd = fixed_inputs.flap;
        context.u_cmd.surface_inputs.spoiler_cmd = fixed_inputs.spoiler;

        // update prior-step control command
        u_cmd_t_1 = context.u_cmd;

        // apply surface actuator dynamics
        context.u_actual.surface_inputs = actuator_properties.step(context.u_cmd.surface_inputs, constants::dt);

        // apply propulsor actuator dynamics
        context.u_actual.propulsor_inputs = actuator_properties.step(context.u_cmd.propulsor_inputs, constants::dt);

        // update prior-step actual control
        u_actual_t_1 = context.u_actual;
    }

    void RunManager::step_physics(StepContext& context) {
        aerodynamics::AerodynamicProperties& aerodynamic_properties = aircraft.aerodynamic_properties;
        actuators::PropulsorActuators& propulsor_actuators = aircraft.actuator_properties.propulsor_actuators;

        integrators::RK4Model rk4_model{
            .struc_t = context.struc_t,
            .aerodynamic = aerodynamic_properties,
            .propulsor_actuators = propulsor_actuators
        };

        operating::OperatingConditions rk4_conditions{
            .atm = context.atm_t,
            .windI = context.windI,
            .steady_state = false
        };

        // compute forces, moments, and next-step rigid body state
        integrators::RK4Output rk4_out = integrators::step_rigid_body_rk4(
            context.Xt,
            rk4_model,
            rk4_conditions,
            context.u_actual,
            constants::dt
        );
        context.Xt1 = rk4_out.Xt1;
        context.WB_net = rk4_out.WB_set.net;
        context.WB_aerodynamic = rk4_out.WB_set.aerodynamic;
        context.WB_propulsive = rk4_out.WB_set.propulsive;
    }

    void RunManager::publish_step(int t, StepContext& context) {
        JSONFlags& json_flags = json_options.flags;

        // update data context
        io::DataContext data_context{
            .Xt=context.Xt,
            .Yt=context.Yt,
            .Zt=context.Zt,
            .u_surface=context.u_actual.surface_inputs,
            .u_propulsor=context.u_actual.propulsor_inputs,
            .u_surface_commanded=context.u_cmd.surface_inputs,
            .u_propulsor_commanded=context.u_cmd.propulsor_inputs,
            .WB_net=context.WB_net,
            .WB_aerodynamic=context.WB_aerodynamic,
            .WB_propulsive=context.WB_propulsive,
            .setpoint=context.setpoint,
            .windB=context.windB
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
                util::print_state(t, context.Xt, context.geo_t, context.aero_t, context.windI);
            }
            scheduler.log_tick -= constants::hz;
        }

        // compute next-step aerodynamic state
        aerodynamics::AerodynamicState aero_t1 = aerodynamics::compute_aerodynamic_state(context.Xt1, context.windB);

        dynamics::Position pB_GB{ context.struc_t.pB_GB.data };
        dynamics::RigidBodyState X_BN_t1 = aircraft.rebase_cg_state(context.Xt1, pB_GB);

        // set step options
        vehicles::StepOptions StepOpts;
        StepOpts.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions{ .X_BN = X_BN_t1 };
        StepOpts.CGFrameFRDStepOpts = vehicles::CGFrameFRDStepOptions{ .pB_GB = pB_GB };
        StepOpts.STABFrameFRDStepOpts = vehicles::STABFrameFRDStepOptions{ .aero = aero_t1 };
        StepOpts.WINDFrameSTABStepOpts = vehicles::WINDFrameSTABStepOptions{ .aero = aero_t1 };

        // step frames
        aircraft.step(StepOpts);

        // advance internal state
        WB_net_t_1 = context.WB_net;

        // compute next-step geographic state
        geography::GeographicState geo_t1 = geography::compute_geographic_state(aircraft.FRDFrameECEF, aircraft.ECEFFrame);

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
    }

    void RunManager::finish_step(fsm::FiniteState current_mode) {
        CLIFlags& cli_flags = cli_options.flags;

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

}
