#include "simulation/propulsion/public/detail/loads.hpp"

namespace propulsion {

    dynamics::Wrench_T<double> compute_propulsive_loads(const actuators::PropulsorActuators& propulsors, const constants::Vector3_T<double>& center_of_gravity, const dynamics::Twist_T<double>& vehicle_twist, const atmospheric::StaticAtmosphericState& atmosphere, const actuators::PropulsorActuatorInputs_T<double>& actuator_inputs, const PropellerOmegaStateSet_T<double>& propeller_state) {
        return compute_propulsive_loads_T<double>(propulsors, center_of_gravity, vehicle_twist, atmosphere, actuator_inputs, propeller_state);
    }

}
