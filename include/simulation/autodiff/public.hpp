#pragma once
#include <cstddef>
#include <Eigen/Dense>
#include <cppad/example/cppad_eigen.hpp>
#include <cppad/cppad.hpp>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/structural/public.hpp"

namespace vehicles { struct Aircraft; } // forward declare

namespace autodiff {

    struct AutoDiffModel {
        const structural::StructuralProperties& structural;
        const aerodynamics::AerodynamicProperties& aerodynamic;
        actuators::PropulsorActuators& propulsor_actuators;
        actuators::ActuatorLimits_T<double> actuator_limits;
        actuators::FixedActuatorInputs fixed_actuator_inputs{};
    };

    AutoDiffModel build_autodiff_model(vehicles::Aircraft& aircraft);

    template <typename T>
    actuators::SurfaceActuatorInputs_T<T> pack_surface_actuator_inputs_T(const actuators::ActuatorInputs_T<T>& u, const actuators::FixedActuatorInputs& fixed_actuator_inputs);

    template <typename T>
    actuators::PropulsorActuatorInputs_T<T> pack_propulsor_actuator_inputs_T(const actuators::ActuatorInputs_T<T>& u);

    template <typename T>
    dynamics::Wrench_T<T> compute_net_wrench_T(const dynamics::State_T<T>& x, const dynamics::Twist_T<T>& twist, const actuators::ActuatorInputs_T<T>& u, AutoDiffModel& model, const operating::OperatingConditions& conditions,T dt);

    template <typename T>
    dynamics::StateDot_T<T> compute_state_dot_T(const dynamics::State_T<T>& x, const actuators::ActuatorInputs_T<T>& u, AutoDiffModel& model, const operating::OperatingConditions& conditions, T dt);

    template <typename T, std::size_t rows>
    Eigen::Matrix<T, rows, 1> eigen_vector_from_cppad_vector(const CppAD::eigen_vector<T>& x);

    template <typename T, int rows>
    CppAD::eigen_vector<T> cppad_vector_from_eigen_vector(const Eigen::Matrix<T, rows, 1>& x);

    template <int rows>
    CppAD::eigen_vector<CppAD::AD<double>> start_autodiff_tracking(const Eigen::Matrix<double, rows, 1>& x);

    template <int output_rows, int input_rows>
    Eigen::Matrix<double, output_rows, input_rows> compute_jac(CppAD::ADFun<double>& f, const Eigen::Matrix<double, input_rows, 1>& x);

}

#include "simulation/autodiff/public.tpp"
