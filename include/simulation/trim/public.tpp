#pragma once
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/propulsion/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/util/public.hpp"
#include "simulation/operating/public.hpp"

namespace trim {

    template <typename T>
    T get_control_from_solver_space_T(const T& u_solver, double limit_min, double limit_max) {
        const T mid = T(0.5 * (limit_max + limit_min));
        const T half_range = T(0.5 * (limit_max - limit_min));
        return mid + half_range * (u_solver / util::sqrt(T(1) + u_solver * u_solver));
    }


    template <typename T>
    actuators::ActuatorInputs_T<T> pack_trim_actuator_inputs_T(const operating::StateInputVector_T<T>& z, const actuators::ActuatorLimits_T<double>& actuator_limits) {
        return {
            .elevator_cmd = get_control_from_solver_space_T<T>(
                z(constants::state_dim + 0), 
                actuator_limits.limit_min.elevator_cmd, actuator_limits.limit_max.elevator_cmd
            ),
            .aileron_cmd = get_control_from_solver_space_T<T>(
                z(constants::state_dim + 1), 
                actuator_limits.limit_min.aileron_cmd, actuator_limits.limit_max.aileron_cmd
            ),
            .rudder_cmd = get_control_from_solver_space_T<T>(
                z(constants::state_dim + 2), 
                actuator_limits.limit_min.rudder_cmd, actuator_limits.limit_max.rudder_cmd
            ),
            .front_propulsor_cmd = get_control_from_solver_space_T<T>(
                z(constants::state_dim + 3), 
                actuator_limits.limit_min.front_propulsor_cmd, actuator_limits.limit_max.front_propulsor_cmd
            ),
            .left_propulsor_cmd = get_control_from_solver_space_T<T>(
                z(constants::state_dim + 4), 
                actuator_limits.limit_min.left_propulsor_cmd, actuator_limits.limit_max.left_propulsor_cmd
            ),
            .right_propulsor_cmd = get_control_from_solver_space_T<T>(
                z(constants::state_dim + 5), 
                actuator_limits.limit_min.right_propulsor_cmd, actuator_limits.limit_max.right_propulsor_cmd
            ),
        };
    }

}
