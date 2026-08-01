#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/autodiff/public.hpp"
#include "simulation/vehicles/public.hpp"


namespace autodiff {

    AutoDiffModel build_autodiff_model(vehicles::Aircraft& aircraft) {
        actuators::SurfaceActuators& surface_actuators = aircraft.actuator_properties.surface_actuators;
        actuators::PropulsorActuators& propulsor_actuators = aircraft.actuator_properties.propulsor_actuators;
        actuators::ActuatorLimits actuator_limits = actuators::pack_actuator_limits(surface_actuators, propulsor_actuators);
        actuators::ActuatorInputsVector_T<double> actuator_time_constants;
        actuator_time_constants << surface_actuators.elevator.tau,
                                   surface_actuators.aileron.tau,
                                   surface_actuators.rudder.tau,
                                   propulsor_actuators.front_propulsor.tau,
                                   propulsor_actuators.left_propulsor.tau,
                                   propulsor_actuators.right_propulsor.tau;

        return {
            .structural = aircraft.structural_properties,
            .aerodynamic = aircraft.aerodynamic_properties,
            .propulsor_actuators = propulsor_actuators,
            .actuator_limits = actuator_limits,
            .actuator_time_constants = actuator_time_constants,
            .fixed_actuator_inputs = aircraft.actuator_properties.settings.get_fixed_actuator_inputs()
        };
    }

}
