#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/autodiff/public.hpp"
#include "simulation/vehicles/public.hpp"
#include "simulation/operating/public.hpp"


namespace autodiff {

    AutoDiffModel build_autodiff_model(vehicles::Aircraft& aircraft) {
        actuators::PropulsorActuators& propulsor_actuators = aircraft.actuator_properties.propulsor_actuators;
        actuators::ActuatorLimits_T<double> actuator_limits = actuators::pack_actuator_limits(aircraft.actuator_properties.surface_actuators, propulsor_actuators);

        return {
            .structural = aircraft.structural_properties,
            .aerodynamic = aircraft.aerodynamic_properties,
            .propulsor_actuators = propulsor_actuators,
            .actuator_limits = actuator_limits,
            .fixed_actuator_inputs = actuators::FixedActuatorInputs{
                .flap = aircraft.operating_properties.fixed_actuator_inputs.flap,
                .spoiler = aircraft.operating_properties.fixed_actuator_inputs.spoiler,
            }
        };
    }

}