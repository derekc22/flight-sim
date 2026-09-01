#include "simulation/avionics/public/data/helpers.hpp"
#include "simulation/avionics/public/manager.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/runner/public/components/measurements.hpp"
#include "simulation/runner/public/components/scheduler.hpp"
#include "simulation/sensors/public/data/helpers.hpp"
#include "simulation/sensors/public/manager.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

namespace runner {

    MeasurementsOutput Measurements::step(const MeasurementsInput& input) {
        // extract reusable quantities
        sensors::SensorManager& sensor_manager = input.aircraft.sensor_manager;
        avionics::AvionicsManager& avionics_manager = input.aircraft.avionics_manager;

        // initialize measurements to ground truth
        dynamics::RigidBodyState Yt = input.context.Xt;

        // aggregate ground truth sensor data
        sensors::SensorGroundTruth sensor_gt = sensors::build_sensor_gt(
            input.context.Xt,
            input.context.XEt,
            input.context.aero_t,
            input.context.atm_t,
            input.context.struc_t.mass,
            input.previous_wrench
        );

        // use sensors and avionics
        sensors::SensorMeasurements sensor_meas;

        if (input.enabled) {
            if (input.scheduler.sensor_tick >= constants::hz) {
                double sensor_dt = input.scheduler.sensor_elapsed_ticks * constants::dt;

                // step sensors
                sensor_meas = sensor_manager.step(sensor_gt, sensor_dt);
                sensor_meas_t_1 = sensor_meas;

                input.scheduler.sensor_tick -= constants::hz;
                input.scheduler.sensor_elapsed_ticks = 0;
            }
            else sensor_meas = sensor_meas_t_1; // perform ZOH
        }

        if (input.enabled) {
            if (input.scheduler.avionics_tick >= constants::hz) {
                double avionics_dt = input.scheduler.avionics_elapsed_ticks * constants::dt;

                // aggregate ground truth avionics data
                avionics::AvionicsGroundTruth avionics_gt = avionics::build_avionics_gt(
                    input.context.Xt,
                    input.context.XEt,
                    input.context.aero_t,
                    input.context.atm_t,
                    input.context.geo_t
                );

                // step avionics
                avionics::AvionicsMeasurements avionics_meas = avionics_manager.step(sensor_meas, sensor_manager.hist, sensor_gt, avionics_gt, avionics_dt);

                // overwrite local measurement state with sensor measurements
                Yt = avionics::get_state_from_avionics(sensor_meas, avionics_meas, avionics_manager.settings);
                Yt_1 = Yt;

                input.scheduler.avionics_tick -= constants::hz;
                input.scheduler.avionics_elapsed_ticks = 0;
            }
            else Yt = Yt_1; // perform ZOH
        }

        return { .measured_state = Yt };
    }

}
