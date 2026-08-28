#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/autodiff/public.hpp"
#include "simulation/vehicles/public.hpp"


namespace autodiff {

    AutoDiffModel build_autodiff_model(vehicles::Aircraft& aircraft, const structural::StructuralState& struc_t) {
        actuators::SurfaceActuators& surface_actuators = aircraft.actuator_manager.surface_actuators;
        actuators::PropulsorActuators& propulsor_actuators = aircraft.actuator_manager.propulsor_actuators;
        actuators::ActuatorLimits actuator_limits = actuators::pack_actuator_limits(surface_actuators, propulsor_actuators);

        return {
            .struc_t = struc_t,
            .aerodynamic = aircraft.aerodynamics_manager,
            .propulsor_actuators = propulsor_actuators,
            .actuator_limits = actuator_limits,
            .fixed_actuator_inputs = aircraft.actuator_manager.settings.get_fixed_actuator_inputs()
        };
    }

}
