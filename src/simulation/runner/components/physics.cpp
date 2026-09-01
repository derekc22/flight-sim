#include "simulation/integrators/public/detail/rk4.hpp"
#include "simulation/runner/public/components/physics.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

namespace runner {

    PhysicsOutput Physics::step(const PhysicsInput& input) {
        aerodynamics::AerodynamicsManager& aerodynamics_manager = input.aircraft.aerodynamics_manager;
        actuators::PropulsorActuators& propulsor_actuators = input.aircraft.actuator_manager.propulsor_actuators;

        integrators::RK4Model rk4_model{
            .struc_t = input.context.struc_t,
            .aerodynamic = aerodynamics_manager,
            .propulsor_actuators = propulsor_actuators,
            .propulsion = input.aircraft.propulsion_manager
        };

        operating::OperatingConditions rk4_conditions{
            .atm = input.context.atm_t,
            .windI = input.context.windI,
            .steady_state = false
        };

        // compute forces, moments, and next-step rigid body state
        integrators::RK4Output rk4_out = integrators::step_rigid_body_rk4(
            input.context.Xt,
            rk4_model,
            rk4_conditions,
            input.context.u_actual,
            constants::dt
        );

        return {
            .next_state = rk4_out.Xt1,
            .net_wrench = rk4_out.WB_set.net,
            .aerodynamic_wrench = rk4_out.WB_set.aerodynamic,
            .propulsive_wrench = rk4_out.WB_set.propulsive
        };
    }

}
