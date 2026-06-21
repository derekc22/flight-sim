#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/propulsion/private.hpp"
#include "simulation/propulsion/public.hpp"

namespace propulsion {

    PropulsiveWrench step_propulsive_forces_moments(actuators::PropulsorActuators& propulsor_actuators, const dynamics::RigidBodyState& X, const atmospheric::StaticAtmosphericState& atm, const actuators::PropulsorActuatorInputs_T<double>& u) {
        PropulsorOmegaDot_T<double> omega_dot{
            .front_propulsor = step_propeller_omega_dot(propulsor_actuators.front_propulsor, u.front_propulsor_cmd, atm),
            .left_propulsor = step_propeller_omega_dot(propulsor_actuators.left_propulsor, u.left_propulsor_cmd, atm),
            .right_propulsor = step_propeller_omega_dot(propulsor_actuators.right_propulsor, u.right_propulsor_cmd, atm)
        };

        dynamics::Twist_T<double> twist{
            .v = X.v.data,
            .w = X.w.data
        };

        dynamics::Wrench_T<double> loads = step_propulsive_forces_moments_T<double>(
            propulsor_actuators,
            twist,
            atm,
            actuators::PropulsorActuatorInputs_T<double>{
                .front_propulsor_cmd = u.front_propulsor_cmd,
                .left_propulsor_cmd = u.left_propulsor_cmd,
                .right_propulsor_cmd = u.right_propulsor_cmd
            },
            omega_dot
        );

        return { dynamics::Force{ loads.F }, dynamics::Moment{ loads.M } };
    }

}
