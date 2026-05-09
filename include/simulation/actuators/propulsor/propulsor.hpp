#pragma once
#include <Eigen/Dense>
#include "simulation/actuators/propulsor/shared.hpp"
#include "simulation/constants/constants.hpp"

namespace actuators {

    template <typename T>
    using FullPropulsorActuatorInputsVector_T = Eigen::Matrix<T, constants::full_propulsor_input_dim, 1>;

    FullPropulsorActuatorInputsVector_T<double> unpack_full_propulsor_actuator_inputs(const PropulsorActuatorInputs_T<double>& u_propulsor);

}
