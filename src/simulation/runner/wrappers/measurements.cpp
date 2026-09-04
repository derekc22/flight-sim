#include "simulation/runner/public/wrappers/measurements.hpp"

#include "simulation/avionics/public/data/helpers.hpp"
#include "simulation/avionics/public/manager.hpp"
#include "simulation/constants/public/scalars.hpp"
#include "simulation/runner/public/scheduling/scheduler.hpp"
#include "simulation/sensors/public/data/helpers.hpp"
#include "simulation/sensors/public/manager.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

namespace runner
{

	MeasurementsWrapperOutput MeasurementsWrapper::step(
		const MeasurementsWrapperInput& input)
	{
		// extract reusable quantities
		sensors::SensorManager& sensor_manager = input.aircraft.sensor_manager;
		avionics::AvionicsManager& avionics_manager = input.aircraft.avionics_manager;

		// initialize measurements to ground truth
		dynamics::RigidBodyState Yt = input.context.Xt;

		// aggregate ground truth sensor data
		sensors::SensorGroundTruth sensor_gt = sensors::build_sensor_gt(input.context.Xt,
			input.context.XEt,
			input.context.aero_t,
			input.context.atm_t,
			input.context.struc_t.mass,
			input.WB_net_t_1);

		// use sensors and avionics
		sensors::SensorMeasurements sensor_meas;

		if (input.enabled) {
			if (input.scheduler.sensor_tick >= constants::hz) {
				double sensor_dt = input.scheduler.sensor_elapsed_ticks * constants::dt;

				// step sensors
				sensor_meas = sensor_manager.step({.sensor_gt = sensor_gt, .dt = sensor_dt}).sensor_meas;
				sensor_meas_t_1 = sensor_meas;

				input.scheduler.sensor_tick -= constants::hz;
				input.scheduler.sensor_elapsed_ticks = 0;
			} else
				sensor_meas = sensor_meas_t_1; // perform ZOH
		}

		if (input.enabled) {
			if (input.scheduler.avionics_tick >= constants::hz) {
				double avionics_dt = input.scheduler.avionics_elapsed_ticks * constants::dt;

				// aggregate ground truth avionics data
				avionics::AvionicsGroundTruth avionics_gt = avionics::build_avionics_gt(input.context.Xt,
					input.context.XEt,
					input.context.aero_t,
					input.context.atm_t,
					input.context.geo_t);

				// step avionics
				// sensor_hist will always be populated by the time AvionicsManager::step is called, but std::optional is kept here for consistency
				avionics::AvionicsMeasurements avionics_meas = avionics_manager
																   .step(
																	   {.sensor_meas = sensor_meas,
																		   .sensor_hist = sensor_manager.hist,
																		   .sensor_gt = sensor_gt,
																		   .avionics_gt = avionics_gt,
																		   .dt = avionics_dt})
																   .avionics_meas;

				// overwrite local measurement state with sensor measurements
				Yt = avionics::get_state_from_avionics(sensor_meas, avionics_meas, avionics_manager.settings);
				Yt_1 = Yt;

				input.scheduler.avionics_tick -= constants::hz;
				input.scheduler.avionics_elapsed_ticks = 0;
			} else
				Yt = Yt_1; // perform ZOH
		}

		return {.Yt = Yt};
	}

} // namespace runner
