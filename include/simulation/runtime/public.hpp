#pragma once
#include "simulation/actuators/public.hpp"

namespace runtime {

	struct RuntimeActuatorProperties {
        actuators::FixedActuatorInputs fixed_actuator_inputs{};
	};

	struct RuntimeAvionicsProperties {
        bool use_gnss = false;
	};

    struct RuntimeProperties {
        RuntimeActuatorProperties runtime_actuator_properties;
        RuntimeAvionicsProperties runtime_avionics_properties;
    };

}