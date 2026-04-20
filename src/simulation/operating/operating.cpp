#include "simulation/operating/operating.hpp"

namespace operating {

    control::SurfaceActuatorInputs fixed_surface_actuator_inputs(const OperatingProperties& operating_properties) {
        return {
            .flap_cmd = operating_properties.fixed_controls.flap,
            .spoiler_cmd = operating_properties.fixed_controls.spoiler,
        };
    }
}
