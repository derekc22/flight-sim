#include <cstddef>
#include <stdexcept>
#include <Eigen/Dense>
#include <cppad/example/cppad_eigen.hpp>
#include <cppad/cppad.hpp>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/integrators/public.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/propulsion/public.hpp"

namespace autodiff {

    template <typename T>
    dynamics::Wrench_T<T> compute_net_wrench_T(const operating::OperatingPoint_T<T>& operating_point, AutoDiffModel& model, const operating::OperatingConditions& conditions, T dt) {
        const dynamics::State_T<T> x = operating_point.state;
        actuators::ActuatorInputs_T<T> u = operating_point.input;

        const dynamics::Twist_T<T> twist = dynamics::build_twist_from_state_T(x);

        u.surface_inputs.flap_cmd = T(model.fixed_actuator_inputs.flap);
        u.surface_inputs.spoiler_cmd = T(model.fixed_actuator_inputs.spoiler);

        // recall, the definition of steady trim is ẋ = f(x, u) = 0
        // that is, for trim, d/dt(·) = 0 must be enforced for all modelled state variables
        // however, despite this repo not including omega as part of the modelled aircraft state, omega_dot is also set to 0
        // this is done because nonzero omega_dot represents a non-steady propeller transient that induces a moment on the aircraft
        // this moment, if unbalanced, will result in ẋ != 0, which violates the condition of steady equilibrium
        // thus, setting omega_dot = 0 allows trim to be achieved
        // note: nonzero omega_dot only violates trim for the reduced-order model if the moment results in any of the modelled aircraft states changing (ẋ != 0)
        // however, for a higher-order model, omega is necessarily included as a modelled aircraft state. as such, omega_dot = 0 is required in this case - regardless of whether it affects other states

        const propulsion::PropellerOmegaDotSet_T<T> propeller_omega_dot_set = conditions.steady_state ?
            propulsion::PropellerOmegaDotSet_T<T>{} : // set omega_dot = 0 if computing gradients for trim (steady state)
            propulsion::compute_propeller_omega_dot_set_T<T>(
                model.propulsor_actuators, 
                u.propulsor_inputs,
                conditions.atm, 
                dt
        );

        const constants::Vector3_T<T> gB = geography::gB_T(x.phi, x.theta);

        const dynamics::WrenchSet_T<T> wrench = integrators::compute_wrench_set_T<T>(
            model,
            twist,
            conditions.atm,
            u,
            propeller_omega_dot_set,
            conditions.windB,
            gB
        );

        return wrench.net;
    }

    template <typename T>
    dynamics::StateDot_T<T> compute_state_dot_T(const operating::OperatingPoint_T<T>& operating_point, AutoDiffModel& model, const operating::OperatingConditions& conditions, T dt) {
        const dynamics::Wrench_T<T> net_wrench = compute_net_wrench_T<T>(operating_point, model, conditions, dt);
        return compute_state_dot_from_net_wrench_T(operating_point.state, model, net_wrench);
    }

    template <typename T>
    dynamics::StateDot_T<T> compute_state_dot_T(const operating::VirtualOperatingPoint_T<T>& operating_point, AutoDiffModel& model) {
        return compute_state_dot_from_net_wrench_T(operating_point.state, model, operating_point.input);
    }

    template <typename T>
    dynamics::StateDot_T<T> compute_state_dot_from_net_wrench_T(const dynamics::State_T<T>& x, AutoDiffModel& model, const dynamics::Wrench_T<T>& net_wrench) {
        const dynamics::Twist_T<T> twist = dynamics::build_twist_from_state_T(x);
        const constants::Vector3_T<T> v_dot = dynamics::ddtB_vB_BI_T<T>(twist.v, twist.w, model.structural.mass.data, net_wrench.F);
        const constants::Vector3_T<T> w_dot = dynamics::ddtB_wB_BI_T<T>(twist.w, model.structural.JB.data, net_wrench.M);
        const constants::Vector3_T<T> eul_dot = dynamics::wB_BI_to_eul_dot_T<T>(twist.w, x.theta, x.phi);

        return {
            .vx_dot = v_dot.x(),
            .vy_dot = v_dot.y(),
            .vz_dot = v_dot.z(),
            .p_dot = w_dot.x(),
            .q_dot = w_dot.y(),
            .r_dot = w_dot.z(),
            .phi_dot = eul_dot.x(),
            .theta_dot = eul_dot.y(),
        };
    }

    template <typename T, std::size_t rows>
    constants::MatrixX_T<T, rows, 1> eigen_vector_from_cppad_vector(const CppAD::eigen_vector<T>& x) {
        if (x.size() != rows) {
            throw std::invalid_argument("autodiff::eigen_vector_from_cppad_vector: vector has incorrect size");
        }

        constants::MatrixX_T<T, rows, 1> out;
        for (std::size_t i = 0; i < rows; ++i) {
            out(static_cast<Eigen::Index>(i)) = x[i];
        }
        return out;
    }

    template <typename T, int rows>
    CppAD::eigen_vector<T> cppad_vector_from_eigen_vector(const constants::MatrixX_T<T, rows, 1>& x) {
        CppAD::eigen_vector<T> out(static_cast<std::size_t>(rows));
        for (int i = 0; i < rows; ++i) {
            out[i] = x(static_cast<Eigen::Index>(i));
        }
        return out;
    }

    template <int rows>
    constants::MatrixX_T<CppAD::AD<double>, rows, 1> double_to_cppad_double(const constants::MatrixX_T<double, rows, 1>& x) {
        return x.template cast<CppAD::AD<double>>();
    }

    template <int rows>
    CppAD::eigen_vector<CppAD::AD<double>> start_autodiff_tracking(const constants::MatrixX_T<double, rows, 1>& x) {
        CppAD::eigen_vector<CppAD::AD<double>> x_tracked = cppad_vector_from_eigen_vector(double_to_cppad_double(x));
        CppAD::Independent(x_tracked);
        return x_tracked;
    }

    template <int output_rows, int input_rows>
    constants::MatrixX_T<double, output_rows, 1> evaluate_tracked_vector(CppAD::ADFun<double>& f, const constants::MatrixX_T<double, input_rows, 1>& x) {
        const CppAD::eigen_vector<double> x_eval = cppad_vector_from_eigen_vector(x);
        const CppAD::eigen_vector<double> y_eval = f.Forward(0, x_eval);
        return eigen_vector_from_cppad_vector<double, output_rows>(y_eval);
    }

    template <int output_rows, int input_rows>
    constants::MatrixX_T<double, output_rows, input_rows> compute_jac(CppAD::ADFun<double>& f, const constants::MatrixX_T<double, input_rows, 1>& x) {
        const CppAD::eigen_vector<double> x_eval = cppad_vector_from_eigen_vector(x);
        const CppAD::eigen_vector<double> jac_flat = f.Jacobian(x_eval);
        const Eigen::Map<const Eigen::Matrix<double, output_rows, input_rows, Eigen::RowMajor>> jac_map(jac_flat.data());
        return constants::MatrixX_T<double, output_rows, input_rows>(jac_map);
    }

}
