#pragma once
#include "simulation/sensors/public/components/air_data/aoa_vane.hpp"
#include "simulation/sensors/public/components/air_data/pitot_tube.hpp"
#include "simulation/sensors/public/components/air_data/static_port.hpp"
#include "simulation/sensors/public/components/air_data/tat_probe.hpp"
#include "simulation/sensors/public/components/inertial/accelerometer.hpp"
#include "simulation/sensors/public/components/inertial/gyroscope.hpp"
#include "simulation/sensors/public/components/navigation/gnss.hpp"
#include "simulation/sensors/public/components/navigation/magnetometer.hpp"

namespace sensors {

	struct Sensors {
		AngleOfAttackVane aoa_vane;
		Accelerometer accelerometer; // IMU
		Gyroscope gyro;              // IMU
		PitotTube pitot_tube;
		StaticPort static_port;
		TotalAirTemperatureProbe tat_probe;
		GNSSReceiver gnss;
		Magnetometer magnetometer;
	};

} // namespace sensors
