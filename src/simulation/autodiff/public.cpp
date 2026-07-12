#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/autodiff/public.hpp"
#include "simulation/vehicles/public.hpp"


namespace autodiff {

    AutoDiffModel build_autodiff_model(vehicles::Aircraft& aircraft) {
        actuators::PropulsorActuators& propulsor_actuators = aircraft.actuator_properties.propulsor_actuators;
        actuators::ActuatorLimits actuator_limits = actuators::pack_actuator_limits(aircraft.actuator_properties.surface_actuators, propulsor_actuators);

        return {
            .structural = aircraft.structural_properties,
            .aerodynamic = aircraft.aerodynamic_properties,
            .propulsor_actuators = propulsor_actuators,
            .actuator_limits = actuator_limits,
            .fixed_actuator_inputs = aircraft.actuator_properties.settings.get_fixed_actuator_inputs()
        };
    }

}
