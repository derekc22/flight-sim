#include "simulation/sensors/public/manager.hpp"

namespace sensors {

    SensorManagerOutput SensorManager::step(const SensorManagerInput& input) {
        SensorMeasurements sensor_meas {
            .alpha = sensors.aoa_vane.measure(input.sensor_gt.alpha, input.dt),
            .fB = sensors.accelerometer.measure(input.sensor_gt.fB, input.dt),
            .wB_BI = sensors.gyro.measure(input.sensor_gt.wB_BI, input.dt),
            .P0 = sensors.pitot_tube.measure(input.sensor_gt.P0, input.dt),
            .P = sensors.static_port.measure(input.sensor_gt.P, input.dt),
            .T0 = sensors.tat_probe.measure(input.sensor_gt.T0, input.dt),
            .pI_BI_gnss = sensors.gnss.measure(input.sensor_gt.pI_BI, input.dt),
            .vB_BI_gnss = sensors.gnss.measure(input.sensor_gt.vB_BI, input.dt),
            .heading_BE = sensors.magnetometer.measure(input.sensor_gt.heading, input.dt)
        };

        hist = sensor_meas;

        return { .sensor_meas = sensor_meas };
    }
}
