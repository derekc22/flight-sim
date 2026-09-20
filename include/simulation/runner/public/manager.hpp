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

		/**
		 * @brief Saves recorded run outputs and configuration snapshots.
		 */
		void cleanup();

		/**
		 * @brief Executes the configured simulation run and saves its outputs.
		 */
		void run();

		/**
		 * @brief Executes one complete simulation step.
		 *
		 * @param[in] input Current simulation-step index.
		 * @return Context containing the inputs, intermediate values, and outputs for the step.
		 */
		RunManagerOutput step(const RunManagerInput& input);

		/**
		 * @brief Builds the current simulation-step context.
		 *
		 * @return Initialized context for the current step.
		 */
		StepContext prepare_step();

		/**
		 * @brief Performs one-time trim and linearization initialization when enabled.
		 *
		 * @param[in,out] context Step context updated with the applied trim state.
		 */
		void initialize_trim(StepContext& context);

		/**
		 * @brief Updates the measured state in a step context.
		 *
		 * @param[in,out] context Step context whose measured state is updated.
		 */
		void step_measurements(StepContext& context);

		/**
		 * @brief Updates the estimated state in a step context.
		 *
		 * @param[in,out] context Step context whose estimated state is updated.
		 */
		void step_estimation(StepContext& context);

		/**
		 * @brief Updates the flight mode, guidance setpoint, and actuator inputs.
		 *
		 * @param[in,out] context Step context updated with control-stage outputs.
		 */
		void step_control(StepContext& context);

		/**
		 * @brief Integrates the aircraft dynamics and updates step-result wrenches.
		 *
		 * @param[in,out] context Step context updated with the next state and wrenches.
		 */
		void step_physics(StepContext& context);

		/**
		 * @brief Records the step and publishes its next aircraft state.
		 *
		 * @param[in] t Simulation-step index [-].
		 * @param[in] context Completed context for the current step.
		 */
		void publish_step(int t, StepContext& context);

		/**
		 * @brief Advances scheduler state and applies real-time pacing.
		 *
		 * @param[in] current_mode Flight mode completed during the current step.
		 */
		void finish_step(fsm::FiniteState current_mode);
	};

} // namespace runner
