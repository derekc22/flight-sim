#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/atmospheric/shared.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/dynamics/shared.hpp"
#include "simulation/propulsion/propulsion.hpp"
#include "simulation/propulsion/shared.hpp"
#include "simulation/util/util.hpp"

namespace propulsion {

    double step_propeller_omega_dot(actuators::PropulsorActuator& propulsor, double thrust, const atmospheric::StaticAtmosphericState& static_atmospheric_state) {
        if (!propulsor.propellers) { return 0.0; }
        const atmospheric::AirDensity& rho = static_atmospheric_state.rho;
        double omega = compute_propeller_omega_T<double>(propulsor, thrust, rho);
        double prev_omega = propulsor.propellers->prev_omega.value_or(omega);
        propulsor.propellers->prev_omega = omega;
        return (omega - prev_omega) / constants::dt;
    }

    PropulsiveWrench step_propulsive_forces_moments(actuators::PropulsorActuators& propulsor_actuators, const dynamics::RigidBodyState& rigid_body_state, const atmospheric::StaticAtmosphericState& static_atmospheric_state, const actuators::PropulsorActuatorInputs_T<double>& u) {
        PropulsorOmegaDot_T<double> omega_dot{
            .front_propulsor = step_propeller_omega_dot(propulsor_actuators.front_propulsor, u.front_propulsor_cmd, static_atmospheric_state),
            .left_propulsor = step_propeller_omega_dot(propulsor_actuators.left_propulsor, u.left_propulsor_cmd, static_atmospheric_state),
            .right_propulsor = step_propeller_omega_dot(propulsor_actuators.right_propulsor, u.right_propulsor_cmd, static_atmospheric_state)
        };

        dynamics::Twist_T<double> twist{
            .v = rigid_body_state.v.data,
            .w = rigid_body_state.w.data
        };

        dynamics::Wrench_T<double> loads = step_propulsive_forces_moments_T<double>(
            propulsor_actuators,
            twist,
            static_atmospheric_state,
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
