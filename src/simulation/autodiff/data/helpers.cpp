#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/actuators/public/manager.hpp"
#include "simulation/autodiff/public/data/helpers.hpp"
#include "simulation/vehicles/public/aircraft.hpp"


namespace autodiff {

    AutoDiffModel build_autodiff_model(vehicles::Aircraft& aircraft, const structural::StructuralState& struc_t) {
        actuators::SurfaceActuators& surface_actuators = aircraft.actuator_manager.surface_actuators;
        actuators::PropulsorActuators& propulsor_actuators = aircraft.actuator_manager.propulsor_actuators;
        actuators::ActuatorLimits actuator_limits = actuators::pack_actuator_limits(surface_actuators, propulsor_actuators);

        return {
            .struc_t = struc_t,
            .aerodynamic = aircraft.aerodynamics_manager,
            .propulsor_actuators = propulsor_actuators,
            .propulsion = aircraft.propulsion_manager,
            .actuator_limits = actuator_limits,
            .fixed_actuator_inputs = aircraft.actuator_manager.settings.fixed_actuator_inputs
        };
    }

}
