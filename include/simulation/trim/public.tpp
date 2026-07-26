#pragma once
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/propulsion/public.hpp"
#include "simulation/trim/public.hpp"
#include "simulation/operating/public.hpp"

namespace trim {

    template <typename T>
    actuators::ActuatorInputs_T<T> pack_trim_actuator_inputs_T(const operating::StateInputVector_T<T>& xu, const actuators::FixedActuatorInputs& fixed_actuator_inputs) {
        actuators::ActuatorInputs_T<T> u = operating::pack_actuator_inputs_T<T>(xu);
        u.surface_inputs.flap_cmd = T(fixed_actuator_inputs.flap);
        u.surface_inputs.spoiler_cmd = T(fixed_actuator_inputs.spoiler);
        return u;
    }

}
