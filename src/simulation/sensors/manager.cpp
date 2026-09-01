#include "simulation/sensors/public/manager.hpp"

namespace sensors {

    SensorManagerOutput SensorManager::step(const SensorManagerInput& input) {
        SensorMeasurements sensor_meas {
            .alpha = sensors.aoa_vane.measure(input.ground_truth.alpha, input.dt),
            .fB = sensors.accelerometer.measure(input.ground_truth.fB, input.dt),
            .wB_BI = sensors.gyro.measure(input.ground_truth.wB_BI, input.dt),
            .P0 = sensors.pitot_tube.measure(input.ground_truth.P0, input.dt),
            .P = sensors.static_port.measure(input.ground_truth.P, input.dt),
            .T0 = sensors.tat_probe.measure(input.ground_truth.T0, input.dt),
            .pI_BI_gnss = sensors.gnss.measure(input.ground_truth.pI_BI, input.dt),
            .vB_BI_gnss = sensors.gnss.measure(input.ground_truth.vB_BI, input.dt),
            .heading_BE = sensors.magnetometer.measure(input.ground_truth.heading, input.dt)
        };

        hist = sensor_meas;

        return { .measurements = sensor_meas };
    }

    SensorMeasurements SensorManager::step(const SensorGroundTruth& sensor_gt, double dt) {
        return step({ .ground_truth = sensor_gt, .dt = dt }).measurements;
    }

}
