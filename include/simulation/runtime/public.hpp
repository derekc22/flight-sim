#pragma once
#include "simulation/actuators/public.hpp"

namespace runtime {

	struct RuntimeActuatorSettings {
        actuators::FixedActuatorInputs fixed_actuator_inputs{};
	};

	struct RuntimeAvionicsSettings {
        bool use_gnss = false;
	};

    struct RuntimeFailureInputs {
        double ground_elev;
        double altitude;
    };

    struct RuntimeProperties {
        RuntimeActuatorSettings runtime_actuator_settings;
        RuntimeAvionicsSettings runtime_avionics_settings;

        void check_runtime_failures(const RuntimeFailureInputs& failure_inputs);
    };

}