#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/propulsion/propulsion.hpp"
#include "simulation/propulsion/public.hpp"

namespace propulsion {

    PropulsiveWrench step_propulsive_forces_moments(actuators::PropulsorActuators& propulsor_actuators, const dynamics::RigidBodyState& rigid_body_state, const atmospheric::StaticAtmosphericState& static_atm_state, const actuators::PropulsorActuatorInputs_T<double>& u) {
        PropulsorOmegaDot_T<double> omega_dot{
            .front_propulsor = step_propeller_omega_dot(propulsor_actuators.front_propulsor, u.front_propulsor_cmd, static_atm_state),
            .left_propulsor = step_propeller_omega_dot(propulsor_actuators.left_propulsor, u.left_propulsor_cmd, static_atm_state),
            .right_propulsor = step_propeller_omega_dot(propulsor_actuators.right_propulsor, u.right_propulsor_cmd, static_atm_state)
        };

        dynamics::Twist_T<double> twist{
            .v = rigid_body_state.v.data,
            .w = rigid_body_state.w.data
        };

        dynamics::Wrench_T<double> loads = step_propulsive_forces_moments_T<double>(
            propulsor_actuators,
            twist,
            static_atm_state,
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
