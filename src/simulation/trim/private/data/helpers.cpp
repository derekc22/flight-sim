#include "simulation/autodiff/public/detail/evaluation.hpp"
#include "simulation/operating/public/data/helpers.hpp"
#include "simulation/trim/private/data/helpers.hpp"
#include "simulation/trim/private/detail/residual.hpp"

namespace trim {

    TrimResidualVector_T<double> fetch_trim_residual_weights(const TrimSolveOptions& options) {
        TrimResidualVector_T<double> w;
        w << 1.0 / options.linear_accel_scale,
             1.0 / options.linear_accel_scale,
             1.0 / options.linear_accel_scale,
             1.0 / options.angular_accel_scale,
             1.0 / options.angular_accel_scale,
             1.0 / options.angular_accel_scale,
             1.0 / options.angle_rate_scale,
             1.0 / options.angle_rate_scale,
             1.0 / options.angle_err_scale,
             1.0 / options.angle_err_scale,
             1.0 / options.angle_err_scale,
             1.0 / options.vel_err_scale,
             1.0 / options.vel_err_scale,
             1.0 / options.angle_rate_scale;
        return w;
    }

    TrimSolution build_trim_solution(const operating::StateInputVector_T<double>& xu, const TrimResidualVector_T<double>& residual, const TrimResidualVector_T<double>& weighted_residual, const autodiff::AutoDiffModel& model, const operating::OperatingConditions& conditions, bool converged, std::size_t iterations) {
        TrimSolution out;
        out.operating_point.state = operating::pack_state_T<double>(xu);
        out.operating_point.input = operating::pack_actuator_inputs_T<double>(xu, model.fixed_actuator_inputs);
        out.conditions = conditions;
        out.wrench = autodiff::compute_net_wrench_T<double>(out.operating_point, model, conditions, constants::dt);
        out.attempted = true;
        out.converged = converged;
        out.iterations = iterations;
        out.residual = pack_trim_residual_T(residual);
        out.weighted_residual = pack_trim_residual_T(weighted_residual);
        // out.residual_norm_2 = residual.norm();
        // out.residual_norm_inf = residual_norm_inf(residual);
        out.weighted_residual_norm_2 = weighted_residual.norm();
        out.weighted_residual_norm_inf = residual_norm_inf(weighted_residual);
        return out;
    }

}
