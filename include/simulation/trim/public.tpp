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
        return mid + half_range * (u_solver / util::sqrt(T(1.0) + u_solver * u_solver));
    }


    template <typename T>
    actuators::ActuatorInputs_T<T> pack_trim_actuator_inputs_T(const operating::StateInputVector_T<T>& xu, const actuators::ActuatorLimits& actuator_limits, const actuators::FixedActuatorInputs& fixed_actuator_inputs) {
        const actuators::SurfaceActuatorInputs_T<double>& surface_limit_min = actuator_limits.limit_min.surface_inputs;
        const actuators::SurfaceActuatorInputs_T<double>& surface_limit_max = actuator_limits.limit_max.surface_inputs;
        const actuators::PropulsorActuatorInputs_T<double>& propulsor_limit_min = actuator_limits.limit_min.propulsor_inputs;
        const actuators::PropulsorActuatorInputs_T<double>& propulsor_limit_max = actuator_limits.limit_max.propulsor_inputs;

        return {
            .surface_inputs = {
                .elevator_cmd = get_control_from_solver_space_T<T>(
                    xu(constants::state_dim + 0),
                    surface_limit_min.elevator_cmd,
                    surface_limit_max.elevator_cmd
                ),
                .aileron_cmd = get_control_from_solver_space_T<T>(
                    xu(constants::state_dim + 1),
                    surface_limit_min.aileron_cmd,
                    surface_limit_max.aileron_cmd
                ),
                .rudder_cmd = get_control_from_solver_space_T<T>(
                    xu(constants::state_dim + 2),
                    surface_limit_min.rudder_cmd,
                    surface_limit_max.rudder_cmd
                ),
                .flap_cmd = T(fixed_actuator_inputs.flap),
                .spoiler_cmd = T(fixed_actuator_inputs.spoiler),
            },
            .propulsor_inputs = {
                .front_propulsor_cmd = get_control_from_solver_space_T<T>(
                    xu(constants::state_dim + 3),
                    propulsor_limit_min.front_propulsor_cmd,
                    propulsor_limit_max.front_propulsor_cmd
                ),
                .left_propulsor_cmd = get_control_from_solver_space_T<T>(
                    xu(constants::state_dim + 4),
                    propulsor_limit_min.left_propulsor_cmd,
                    propulsor_limit_max.left_propulsor_cmd
                ),
                .right_propulsor_cmd = get_control_from_solver_space_T<T>(
                    xu(constants::state_dim + 5),
                    propulsor_limit_min.right_propulsor_cmd,
                    propulsor_limit_max.right_propulsor_cmd
                ),
            }
        };
    }

}
