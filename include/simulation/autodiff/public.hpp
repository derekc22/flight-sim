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
        actuators::ActuatorLimits actuator_limits;
        actuators::ActuatorInputsVector_T<double> actuator_time_constants;
        actuators::FixedActuatorInputs fixed_actuator_inputs;
    };

    AutoDiffModel build_autodiff_model(vehicles::Aircraft& aircraft);

    template <typename T>
    dynamics::Wrench_T<T> compute_net_wrench_T(const operating::OperatingPoint_T<T>& operating_point, AutoDiffModel& model, const operating::OperatingConditions& conditions, T dt);

    template <typename T>
    dynamics::StateDot_T<T> compute_state_dot_T(const operating::OperatingPoint_T<T>& operating_point, AutoDiffModel& model, const operating::OperatingConditions& conditions, T dt);

    template <typename T>
    dynamics::StateDot_T<T> compute_state_dot_T(const operating::VirtualOperatingPoint_T<T>& operating_point, AutoDiffModel& model);

    template <typename T>
    dynamics::StateDot_T<T> compute_state_dot_from_net_wrench_T(const dynamics::State_T<T>& x, AutoDiffModel& model, const dynamics::Wrench_T<T>& net_wrench);

    template <typename T, std::size_t rows>
    constants::MatrixX_T<T, rows, 1> eigen_vector_from_cppad_vector(const CppAD::eigen_vector<T>& x);

    template <typename T, int rows>
    CppAD::eigen_vector<T> cppad_vector_from_eigen_vector(const constants::MatrixX_T<T, rows, 1>& x);

    template <int rows>
    constants::MatrixX_T<CppAD::AD<double>, rows, 1> double_to_cppad_double(const constants::MatrixX_T<double, rows, 1>& x);

    template <int rows>
    CppAD::eigen_vector<CppAD::AD<double>> start_autodiff_tracking(const constants::MatrixX_T<double, rows, 1>& x);

    template <int output_rows, int input_rows>
    constants::MatrixX_T<double, output_rows, 1> evaluate_tracked_vector(CppAD::ADFun<double>& f, const constants::MatrixX_T<double, input_rows, 1>& x);

    template <int output_rows, int input_rows>
    constants::MatrixX_T<double, output_rows, input_rows> compute_jac(CppAD::ADFun<double>& f, const constants::MatrixX_T<double, input_rows, 1>& x);

}

#include "simulation/autodiff/public.tpp"
