#include <cmath>
#include "simulation/constants/public/scalars.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/constants/public/dimensions.hpp"
#include "simulation/actuators/public/data/helpers.hpp"
#include "simulation/actuators/public/manager.hpp"
#include "simulation/allocator/private/detail/effectiveness.hpp"
#include "simulation/allocator/private/detail/solve.hpp"
#include "simulation/allocator/public/manager.hpp"
#include "simulation/autodiff/public/detail/evaluation.hpp"
#include "simulation/dynamics/public/data/helpers.hpp"

namespace allocator {

    AllocatorManagerOutput AllocatorManager::step(const AllocatorManagerInput& input) {

        auto [E, mu_0] = compute_effectiveness_matrix(input.model, input.operating_point, input.conditions);

        const actuators::ActuatorInputsVector_T<double> u_0 = actuators::unpack_actuator_inputs_T(input.operating_point.input);

        dynamics::WrenchVector_T<double> mu_actuator_0 = mu_0;
        if (input.u_preferred.has_value()) {
            const operating::OperatingPoint_T<double> preferred_operating_point{
                .state = input.operating_point.state,
                .input = input.u_preferred.value()
            };
            const dynamics::Wrench_T<double> mu_preferred = autodiff::compute_net_wrench_T<double>(preferred_operating_point, input.model, input.conditions, constants::dt);
            mu_actuator_0 -= dynamics::unpack_wrench_T(mu_preferred);
        }

        dynamics::WrenchVector_T<double> err = input.mu - mu_actuator_0;
        EffectivenessMatrix E_active = E;

        // evaluate active mask
        for (Eigen::Index i = 0; i < err.rows(); ++i) {
            if (!input.active_mask[i]) {
                err(i) = 0.0;
                E_active.row(i).setZero();
            }
        }

        constants::MatrixX_T<double, constants::input_dim, constants::input_dim> trim_hessian = constants::MatrixX_T<double, constants::input_dim, constants::input_dim>::Zero();
        actuators::ActuatorInputsVector_T<double> trim_gradient = actuators::ActuatorInputsVector_T<double>::Zero();
        actuators::ActuatorInputsVector_T<double> actuator_target = u_0;
        if (input.u_preferred.has_value()) {
            const actuators::ActuatorInputsVector_T<double> u_preferred = actuators::unpack_actuator_inputs_T(input.u_preferred.value());
            trim_hessian = R;
            trim_gradient = R * (u_0 - u_preferred);
            actuator_target = u_preferred;
        }

        const constants::MatrixX_T<double, constants::input_dim, constants::input_dim> hessian = E_active.transpose() * Q * E_active + R + trim_hessian;
        const actuators::ActuatorInputsVector_T<double> gradient = -E_active.transpose() * Q * err + trim_gradient;
        const actuators::ActuatorLimitsVector limits = actuators::unpack_actuator_limits(input.model.actuator_limits);

        control::ControlOutput u_constrained = solve_qp_constrained(solver, hessian, gradient, u_0, actuator_target, limits, input.actuator_mask);
        control::ControlOutput u_unconstrained = solve_qp_unconstrained(hessian, gradient, u_0, actuator_target, limits, input.actuator_mask);

        actuators::ActuatorInputsVector_T<double> u_constrained_vec = actuators::unpack_actuator_inputs_T(u_constrained);
        actuators::ActuatorInputsVector_T<double> u_unconstrained_vec = actuators::unpack_actuator_inputs_T(u_unconstrained);

        bool allocation_limited = false;
        for (Eigen::Index i = 0; i < u_unconstrained_vec.rows(); ++i) {
            if (!input.actuator_mask[i] || std::abs(limits(i, 1) - limits(i, 0)) <= constants::eps) {
                continue;
            }

            const double tolerance = constants::eps * (1.0 + std::abs(limits(i, 0)) + std::abs(limits(i, 1)));
            if (u_unconstrained_vec(i) < limits(i, 0) - tolerance || u_unconstrained_vec(i) > limits(i, 1) + tolerance) {
                allocation_limited = true;
                break;
            }
        }

        dynamics::WrenchVector_T<double> delta_mu_vec = dynamics::WrenchVector_T<double>::Zero();
        if (allocation_limited) {
            delta_mu_vec = E_active * (u_constrained_vec - u_unconstrained_vec);
        }

        return {
            .u = u_constrained,
            .delta_mu_vec_t_1 = delta_mu_vec
        };
    }

}
