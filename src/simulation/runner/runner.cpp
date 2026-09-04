#include "simulation/runner/public/runner.hpp"

#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/aerodynamics/public/data/helpers.hpp"
#include "simulation/aerodynamics/public/detail/state.hpp"
#include "simulation/atmospheric/public/detail/state.hpp"
#include "simulation/autodiff/public/data/helpers.hpp"
#include "simulation/dynamics/public/detail/state.hpp"
#include "simulation/geography/public/detail/geodesy.hpp"
#include "simulation/runner/private/data/helpers.hpp"
#include "simulation/trim/public/data/helpers.hpp"
#include "simulation/trim/public/detail/application.hpp"

#include <array>
#include <chrono>
#include <string>

namespace runner
{

	Runner::Runner(
		const CLIOptions& cli_options,
		const JSONOptions& json_options)
		: cli_options(cli_options),
		  json_options(json_options),

		  // load vehicle
		  aircraft(load_vehicle(cli_options.aircraft_id, json_options.flags)),

		  // create state machine
		  fsm_manager(
			  {.trim_flag = json_options.flags.trim_flag,
				  .control_flag = json_options.flags.control_flag,
				  .joystick_flag = json_options.flags.joystick_flag}),

		  // initialize scheduler
		  scheduler(json_options.module_rates, json_options.tf),

		  control_wrapper(json_options.flags,
			  aircraft.actuator_manager.surface_actuators,
			  aircraft.actuator_manager.propulsor_actuators),

		  // initialize udp connections
		  flight_gear_adapter{},

		  recording_wrapper(cli_options, json_options)
	{
		// start timer
		scheduler.next = std::chrono::steady_clock::now();
	}

	Runner::~Runner() = default;

	void Runner::cleanup()
	{
		recording_wrapper.cleanup(cli_options);
	}

	void Runner::run()
	{
		for (int t = 0; t < json_options.tf; ++t) {
			step({.t = t});
		}
		cleanup();
	}

	RunnerOutput Runner::step(
		const RunnerInput& input)
	{
		StepContext context = prepare_step();
		initialize_trim(context);
		step_measurements(context);
		step_estimation(context);
		step_control(context);
		step_physics(context);
		publish_step(input.t, context);
		finish_step(context.current_mode);
		return {.context = context};
	}

	StepContext Runner::prepare_step()
	{
		JSONFlags& json_flags = json_options.flags;

		FlightGearAdapterOutput flight_gear_output =
			flight_gear_adapter.receive({.aircraft = aircraft, .wind_flag = json_flags.wind_flag});
		atmospheric::Wind windI = flight_gear_output.windI;
		atmospheric::Wind windB = flight_gear_output.windB;

		// compute rigid body states
		dynamics::RigidBodyState Xt = dynamics::compute_rigid_body_state(aircraft.CGFrameFRD, aircraft.NEDFrameECEF);
		dynamics::RigidBodyState XEt = dynamics::compute_rigid_body_state(aircraft.CGFrameFRD, aircraft.ECEFFrame);

		// compute structural state
		structural::StructuralState struc_t = aircraft.structural_manager.step({}).struc_t;

		// compute aerodynamic state
		aerodynamics::AerodynamicState aero_t =
			aerodynamics::compute_aerodynamic_state(aircraft.CGFrameFRD, aircraft.NEDFrameECEF, windB);

		// compute geographic state
		geography::GeographicState geo_t =
			geography::compute_geographic_state(aircraft.FRDFrameECEF, aircraft.ECEFFrame);

		// compute static atmospheric state
		atmospheric::StaticAtmosphericState atm_t =
			atmospheric::compute_static_atmospheric_state(aircraft.FRDFrameECEF, aircraft.ECEFFrame);

		// build autodiff model
		autodiff::AutoDiffModel autodiff_model = autodiff::build_autodiff_model(aircraft, struc_t);

		// initialize transient conditions
		operating::OperatingConditions transient_conditions{.atm = atm_t, .windB = windB, .steady_state = false};

		return StepContext{
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

	void Runner::initialize_trim(
		StepContext& context)
	{
		// get aircraft properties
		actuators::ActuatorManager& actuator_manager = aircraft.actuator_manager;
		actuators::SurfaceActuators& surface_actuators = actuator_manager.surface_actuators;
		actuators::PropulsorActuators& propulsor_actuators = actuator_manager.propulsor_actuators;

		// get flags
		JSONFlags& json_flags = json_options.flags;

		// trim and linearization
		if (json_flags.trim_flag && !trim_wrapper.trim_sol.attempted) {
			TrimWrapperOutput trim_output = trim_wrapper.step(
				{.aircraft = aircraft, .autodiff_model = context.autodiff_model, .windB = context.windB});

			// update analysis context
			if (recording_wrapper.analysis_manager) {
				recording_wrapper.analysis_manager->context.trim_sol = trim_output.trim_sol;
			}

			if (trim_output.trim_sol.converged) {
				// obtain full state from trim solution
				dynamics::RigidBodyState Xt_trim =
					trim::update_state_from_trim(context.Xt, trim_output.trim_sol.operating_point.state);
				aerodynamics::AerodynamicState aero_t_trim =
					aerodynamics::compute_aerodynamic_state(Xt_trim, trim_output.trim_sol.conditions.windB);

				dynamics::Position pB_GB{context.struc_t.pB_GB.data};
				dynamics::RigidBodyState X_BN_trim = dynamics::rebase_cg_state(Xt_trim, pB_GB, aircraft.CGFrameFRD);

				// define TrimStepOptions
				vehicles::StepOptions TrimStepOptions;

				// overwrite state with trim state
				TrimStepOptions.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions{.X_BN = X_BN_trim};
				TrimStepOptions.CGFrameFRDStepOpts = vehicles::CGFrameFRDStepOptions{.pB_GB = pB_GB};
				TrimStepOptions.STABFrameFRDStepOpts = vehicles::STABFrameFRDStepOptions{.aero = aero_t_trim};
				TrimStepOptions.WINDFrameSTABStepOpts = vehicles::WINDFrameSTABStepOptions{.aero = aero_t_trim};

				// step frames
				aircraft.step(TrimStepOptions);

				// overwrite local state with trim state
				context.Xt = Xt_trim;
				context.XEt = dynamics::compute_rigid_body_state(aircraft.CGFrameFRD, aircraft.ECEFFrame);
				context.aero_t = aerodynamics::compute_aerodynamic_state(context.Xt, context.windB);
				context.geo_t = geography::compute_geographic_state(aircraft.FRDFrameECEF, aircraft.ECEFFrame);
				context.atm_t =
					atmospheric::compute_static_atmospheric_state(aircraft.FRDFrameECEF, aircraft.ECEFFrame);

				dynamics::Wrench WB_net_trim = {
					.F = dynamics::Force{trim_output.trim_sol.wrench.F},
					.M = dynamics::Moment{trim_output.trim_sol.wrench.M},
				};

				// overwrite internal state with trim state
				physics_wrapper.WB_net_t_1 = WB_net_trim;

				// overwrite actuator lag state with trim controls
				trim::update_actuators_lag_from_trim(surface_actuators, propulsor_actuators, trim_output.trim_sol);
				control_wrapper.u_actual_t_1 = trim_output.trim_sol.operating_point.input;

				LinearizationWrapperOutput linearization_output = linearization_wrapper.step(
					{.autodiff_model = context.autodiff_model, .trim_sol = trim_output.trim_sol});

				// update analysis context
				if (recording_wrapper.analysis_manager) {
					recording_wrapper.analysis_manager->context.lin_sol = linearization_output.lin_sol;
					recording_wrapper.analysis_manager->context.eig_sol = linearization_output.eig_sol;
				}
			}
		}
	}

	void Runner::step_measurements(
		StepContext& context)
	{
		MeasurementsWrapperOutput output = measurements_wrapper.step(
			{.aircraft = aircraft,
				.scheduler = scheduler,
				.context = context,
				.WB_net_t_1 = physics_wrapper.WB_net_t_1,
				.avionics_flag = json_options.flags.avionics_flag});
		context.Yt = output.Yt;
	}

	void Runner::step_estimation(
		StepContext& context)
	{
		EstimationWrapperOutput output = estimation_wrapper.step(
			{.aircraft = aircraft,
				.scheduler = scheduler,
				.context = context,
				.trim_sol = trim_wrapper.trim_sol,
				.lin_sol = linearization_wrapper.lin_sol,
				.u_actual_t_1 = control_wrapper.u_actual_t_1,
				.estimation_flag = json_options.flags.estimation_flag});
		context.Zt = output.Zt;
	}

	void Runner::step_control(
		StepContext& context)
	{
		devices::JoystickManagerOutput joystick_output = control_wrapper.poll_joystick();

		// step state machine
		context.current_mode = fsm_manager.step({.mode_toggled = joystick_output.mode_toggled}).current_mode;

		ControlWrapperOutput output = control_wrapper.step(
			{.aircraft = aircraft,
				.scheduler = scheduler,
				.context = context,
				.trim_sol = trim_wrapper.trim_sol,
				.virtual_lin_sol = linearization_wrapper.virtual_lin_sol,
				.joystick_output = joystick_output,
				.current_mode = context.current_mode});
		context.setpoint = output.setpoint;
		context.u_cmd = output.u_cmd;
		context.u_actual = output.u_actual;
	}

	void Runner::step_physics(
		StepContext& context)
	{
		PhysicsWrapperOutput output = physics_wrapper.step({.aircraft = aircraft, .context = context});
		context.Xt1 = output.Xt1;
		context.WB_net = output.WB_net;
		context.WB_aerodynamic = output.WB_aerodynamic;
		context.WB_propulsive = output.WB_propulsive;
	}

	void Runner::publish_step(
		int t,
		StepContext& context)
	{
		recording_wrapper.step({.t = t, .context = context, .scheduler = scheduler, .flags = json_options.flags});

		// compute next-step aerodynamic state
		aerodynamics::AerodynamicState aero_t1 = aerodynamics::compute_aerodynamic_state(context.Xt1, context.windB);

		dynamics::Position pB_GB{context.struc_t.pB_GB.data};
		dynamics::RigidBodyState X_BN_t1 = dynamics::rebase_cg_state(context.Xt1, pB_GB, aircraft.CGFrameFRD);

		// set step options
		vehicles::StepOptions StepOpts;
		StepOpts.FRDFrameNEDStepOpts = vehicles::FRDFrameNEDStepOptions{.X_BN = X_BN_t1};
		StepOpts.CGFrameFRDStepOpts = vehicles::CGFrameFRDStepOptions{.pB_GB = pB_GB};
		StepOpts.STABFrameFRDStepOpts = vehicles::STABFrameFRDStepOptions{.aero = aero_t1};
		StepOpts.WINDFrameSTABStepOpts = vehicles::WINDFrameSTABStepOptions{.aero = aero_t1};

		// step frames
		aircraft.step(StepOpts);

		// advance internal state
		physics_wrapper.WB_net_t_1 = context.WB_net;

		// compute next-step geographic state
		geography::GeographicState geo_t1 =
			geography::compute_geographic_state(aircraft.FRDFrameECEF, aircraft.ECEFFrame);

		// check for runtime failures
		geography::HeightAGL height_agl{geo_t1.alt.data - flight_gear_adapter.cached_msg_out.ground_elevation.data};
		failures::FailureManagerInput failure_inputs{.height_agl = height_agl};
		failure_manager.step(failure_inputs);

		flight_gear_adapter.send({.geo_t1 = geo_t1, .eulNB = aircraft.FRDFrameNED.eulNB});
	}

	void Runner::finish_step(
		fsm::FiniteState current_mode)
	{
		CLIFlags& cli_flags = cli_options.flags;

		// step scheduler
		scheduler.step({.current_mode = current_mode, .fast_flag = cli_flags.fast_flag});
	}

} // namespace runner
