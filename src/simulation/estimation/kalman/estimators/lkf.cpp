#include "simulation/control/control.hpp"
#include "simulation/estimation/kalman/estimators/lkf.hpp"

namespace estimation {

    LinearKalmanFilter::LinearKalmanFilter(const LinearKalmanFilterParameters& params) : params(params), policy(params) {}

    Eigen::VectorXd LinearKalmanFilter::make_measurement_deviation(const KalmanFilterInput& kalman_filter_input) const {
        return trim::unpack_rigid_body_state(kalman_filter_input.zN_t) - trim::unpack_trim_state_T(kalman_filter_input.trim_sol.state);
    }

    Eigen::VectorXd LinearKalmanFilter::make_input_deviation(const KalmanFilterInput& kalman_filter_input) const {
        return control::unpack_actuator_inputs(kalman_filter_input.u_surface_actual_prev, kalman_filter_input.u_propulsor_actual_prev) - trim::unpack_trim_actuator_inputs_T(kalman_filter_input.trim_sol.input);
    }

    dynamics::RigidBodyState LinearKalmanFilter::pack_state_estimate(const KalmanFilterInput& kalman_filter_input, const Eigen::VectorXd& x_est_deviation) const {
        const trim::TrimStateVector_T<double> x_est = trim::unpack_trim_state_T(kalman_filter_input.trim_sol.state) + x_est_deviation;
        dynamics::RigidBodyState state_estimate = kalman_filter_input.zN_t;

        dynamics::EulerAngles eul_meas;
        eul_meas.set(kalman_filter_input.zN_t.q);

        dynamics::EulerAngles eul_est{ Eigen::Vector3d(eul_meas.psi(), x_est(7), x_est(6)) };
        dynamics::OrientationQuaternion q_est;
        q_est.set(eul_est);

        state_estimate.v = dynamics::LinearVelocity{ x_est.segment<3>(0) };
        state_estimate.w = dynamics::AngularVelocity{ x_est.segment<3>(3) };
        state_estimate.q = q_est;
        return state_estimate;
    }

    EstimationOutput LinearKalmanFilter::step(const KalmanFilterInput& kalman_filter_input) {
        const linearization::DiscretizedTrimLinearization lin_sol_d = linearization::discretize(kalman_filter_input.lin_sol);
        const Eigen::VectorXd zt_deviation = make_measurement_deviation(kalman_filter_input);
        const Eigen::VectorXd ut_1_deviation = make_input_deviation(kalman_filter_input);

        if (!initialized) {
            state = KalmanState{ .x = zt_deviation, .P = params.P0 };
            initialized = true;
        }

        state = policy.correct(policy.predict(state, lin_sol_d.A, lin_sol_d.B, ut_1_deviation), zt_deviation);
        return { .state_estimate = pack_state_estimate(kalman_filter_input, state.x) };
    }
}
