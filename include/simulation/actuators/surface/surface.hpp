#pragma once
#include <Eigen/Dense>
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/constants/constants.hpp"

namespace actuators {

    template <typename T>
    using FullSurfaceActuatorInputsVector_T = Eigen::Matrix<T, constants::full_surface_input_dim, 1>;

    FullSurfaceActuatorInputsVector_T<double> unpack_full_surface_actuator_inputs(const SurfaceActuatorInputs_T<double>& u_surface);

}
