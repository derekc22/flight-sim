#pragma once
#include "simulation/failures/public/manager.hpp"
#include "simulation/fsm/public/manager.hpp"
#include "simulation/runner/public/components/flight_gear.hpp"
#include "simulation/runner/public/components/scheduler.hpp"
#include "simulation/runner/public/components/wrappers/control.hpp"
#include "simulation/runner/public/components/wrappers/estimation.hpp"
#include "simulation/runner/public/components/wrappers/linearization.hpp"
#include "simulation/runner/public/components/wrappers/measurements.hpp"
#include "simulation/runner/public/components/wrappers/physics.hpp"
#include "simulation/runner/public/components/wrappers/recording.hpp"
#include "simulation/runner/public/components/wrappers/trim.hpp"
#include "simulation/runner/public/data/types.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

namespace runner
{

	struct RunManager {
		CLIOptions cli_options;
		JSONOptions json_options;
		vehicles::Aircraft aircraft;
		failures::FailureManager failure_manager;

		// initialize state machine
		fsm::FSMManager fsm_manager;

		// initialize scheduler
		Scheduler scheduler;

		// initialize trim and linearization solutions
		TrimWrapper trim_wrapper;
		LinearizationWrapper linearization_wrapper;

		// initialize prior-step values to perform ZOH
		MeasurementsWrapper measurements_wrapper;
		EstimationWrapper estimation_wrapper;
		ControlWrapper control_wrapper;
		PhysicsWrapper physics_wrapper;
		FlightGearAdapter flight_gear_adapter;
		RecordingWrapper recording_wrapper;

		RunManager(const CLIOptions& cli_options, const JSONOptions& json_options);
		~RunManager();

		void cleanup();
		void run();
		RunManagerOutput step(const RunManagerInput& input);

		StepContext prepare_step();
		void initialize_trim(StepContext& context);
		void step_measurements(StepContext& context);
		void step_estimation(StepContext& context);
		void step_control(StepContext& context);
		void step_physics(StepContext& context);
		void publish_step(int t, StepContext& context);
		void finish_step(fsm::FiniteState current_mode);
	};

} // namespace runner
