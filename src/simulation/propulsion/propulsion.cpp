#include "simulation/propulsion/propulsion.hpp"
#include "simulation/util/util.hpp"
#include "simulation/control/control.hpp"

namespace propulsion {

    PropulsiveWrench step_propulsive_forces_moments(const actuators::PropulsorActuators& propulsor_actuators, const control::PropulsorActuatorInputs& u) {

        PropulsiveWrench_T<double> loads = step_propulsive_forces_moments_T<double>(
            propulsor_actuators,
            PropulsorActuatorInputs_T<double>{
                .front_propulsor_cmd = u.front_propulsor_cmd,
                .left_propulsor_cmd = u.left_propulsor_cmd,
                .right_propulsor_cmd = u.right_propulsor_cmd
            }
        );

        return { dynamics::Force{ loads.F }, dynamics::Moment{ loads.M } };

    }

}