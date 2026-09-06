#include "simulation/runner/public/components/scheduler.hpp"

#include "simulation/constants/public/scalars.hpp"

#include <cmath>
#include <thread>

namespace runner
{

	Scheduler::Scheduler(
		const ModuleRates& module_rates,
		int tf)
		: module_rates(module_rates)
	{
		// fraction of steps that will call guidance
		double frac_guidance_steps = module_rates.guidance_hz / constants::hz;
		// for the immediate call at t=0 + number of remaining steps that will call guidance
		guidance_tf = 1 + static_cast<int>(std::floor((tf - 1) * frac_guidance_steps));
	}

	void Scheduler::step(
		const SchedulerInput& input)
	{
		sensor_tick += module_rates.sensor_hz;
		++sensor_elapsed_ticks;

		avionics_tick += module_rates.avionics_hz;
		++avionics_elapsed_ticks;

		estimation_tick += module_rates.estimation_hz;
		++estimation_elapsed_ticks;

		if (input.current_mode == fsm::FiniteState::Autopilot) {
			guidance_tick += module_rates.guidance_hz;
			control_tick += module_rates.control_hz;
			++control_elapsed_ticks;
		}

		log_tick += module_rates.log_hz;

		// step timer by dt
		next += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
			std::chrono::duration<double>(constants::dt));

		// sleep to maintain frequency dictated by dt
		if (!input.fast_flag) {
			std::this_thread::sleep_until(next);
		}
	}

} // namespace runner
