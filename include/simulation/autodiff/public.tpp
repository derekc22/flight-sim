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
#include "simulation/operating/public.hpp"
#include "simulation/propulsion/public.hpp"

namespace autodiff {

    template <typename T>
    actuators::SurfaceActuatorInputs_T<T> pack_surface_actuator_inputs_T(const actuators::ActuatorInputs_T<T>& u, const actuators::FixedActuatorInputs& fixed_actuator_inputs) {
        return {
            .elevator_cmd = u.elevator_cmd,
            .aileron_cmd = u.aileron_cmd,
            .rudder_cmd = u.rudder_cmd,
            .flap_cmd = T(fixed_actuator_inputs.flap),
            .spoiler_cmd = T(fixed_actuator_inputs.spoiler),
        };
    }

    template <typename T>
    actuators::PropulsorActuatorInputs_T<T> pack_propulsor_actuator_inputs_T(const actuators::ActuatorInputs_T<T>& u) {
        return {
            .front_propulsor_cmd = u.front_propulsor_cmd,
            .left_propulsor_cmd = u.left_propulsor_cmd,
            .right_propulsor_cmd = u.right_propulsor_cmd
        };
    }

    template <typename T>
    dynamics::Wrench_T<T> compute_net_wrench_T(
        const dynamics::State_T<T>& x, 
        const dynamics::Twist_T<T>& twist, 
        const actuators::ActuatorInputs_T<T>& u, 
        AutoDiffModel& model, 
        const operating::OperatingConditions& conditions,
        T dt
    ) {
        const actuators::SurfaceActuatorInputs_T<T> surface_actuator_inputs = pack_surface_actuator_inputs_T(u, model.fixed_actuator_inputs);
        const dynamics::Wrench_T<T> aero_wrench = aerodynamics::step_aero_forces_moments_T<T>(
            model.aerodynamic, 
            twist, 
            conditions.atm, 
            surface_actuator_inputs, 
            conditions.windB
        );

        const actuators::PropulsorActuatorInputs_T<T> propulsor_actuator_inputs = pack_propulsor_actuator_inputs_T(u);
        const dynamics::Wrench_T<T> prop_wrench = propulsion::step_propulsive_forces_moments_T<T>(
            model.propulsor_actuators,
            twist,
            conditions.atm, 
            propulsor_actuator_inputs, 
            dt,
            conditions.steady_state
        );

        return {
            .F = aero_wrench.F + prop_wrench.F + T(model.structural.mass.data) * geography::gB_T(x.phi, x.theta),
            .M = aero_wrench.M + prop_wrench.M,
        };
    }

    template <typename T>
    dynamics::StateDot_T<T> compute_state_dot_T(
        const dynamics::State_T<T>& x, 
        const actuators::ActuatorInputs_T<T>& u, 
        AutoDiffModel& model, 
        const operating::OperatingConditions& conditions, 
        T dt
    ) {
        const dynamics::Twist_T<T> twist = dynamics::build_twist_from_state_T(x);
        const dynamics::Wrench_T<T> net_wrench = compute_net_wrench_T<T>(x, twist, u, model, conditions, dt);
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
    Eigen::Matrix<T, rows, 1> eigen_vector_from_cppad_vector(const CppAD::eigen_vector<T>& x) {
        if (x.size() != rows) {
            throw std::invalid_argument("autodiff::eigen_vector_from_cppad_vector: vector has incorrect size");
        }

        Eigen::Matrix<T, rows, 1> out;
        for (std::size_t i = 0; i < rows; ++i) {
            out(static_cast<Eigen::Index>(i)) = x[i];
        }
        return out;
    }

    template <typename T, int rows>
    CppAD::eigen_vector<T> cppad_vector_from_eigen_vector(const Eigen::Matrix<T, rows, 1>& x) {
        CppAD::eigen_vector<T> out(static_cast<std::size_t>(rows));
        for (int i = 0; i < rows; ++i) {
            out[i] = x(static_cast<Eigen::Index>(i));
        }
        return out;
    }

    template <int rows>
    CppAD::eigen_vector<CppAD::AD<double>> start_autodiff_tracking(const Eigen::Matrix<double, rows, 1>& x) {
        CppAD::eigen_vector<CppAD::AD<double>> x_tracked = cppad_vector_from_eigen_vector(Eigen::Matrix<CppAD::AD<double>, rows, 1>(x.template cast<CppAD::AD<double>>()));
        CppAD::Independent(x_tracked);
        return x_tracked;
    }

    template <int output_rows, int input_rows>
    Eigen::Matrix<double, output_rows, input_rows> compute_jac(CppAD::ADFun<double>& f, const Eigen::Matrix<double, input_rows, 1>& x) {
        const CppAD::eigen_vector<double> x_eval = cppad_vector_from_eigen_vector(x);
        const CppAD::eigen_vector<double> jac_flat = f.Jacobian(x_eval);
        const Eigen::Map<const Eigen::Matrix<double, output_rows, input_rows, Eigen::RowMajor>> jac_map(jac_flat.data());
        return Eigen::Matrix<double, output_rows, input_rows>(jac_map);
    }

}
