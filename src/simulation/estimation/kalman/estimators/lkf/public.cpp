#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/estimation/kalman/estimators/lkf/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/trim/public.hpp"

namespace estimation {

    LinearKalmanFilter::LinearKalmanFilter(const LinearKalmanFilterParameters& params) : params(params), policy(params) {}

    dynamics::StateVector_T<double> LinearKalmanFilter::makemeasurement_deviation(const KalmanFilterInput& estimator_input) {
        return dynamics::unpack_rigid_body_state(estimator_input.yN_t) - dynamics::unpack_state_T(estimator_input.trim_sol.state);
    }

    actuators::ActuatorInputsVector_T<double> LinearKalmanFilter::make_input_deviation(const KalmanFilterInput& estimator_input) {
        return actuators::unpack_actuator_inputs(estimator_input.u_surface_actual_prev, estimator_input.u_propulsor_actual_prev) - actuators::unpack_actuator_inputs_T(estimator_input.trim_sol.input);
    }

    dynamics::RigidBodyState LinearKalmanFilter::pack_state_estimate(const KalmanFilterInput& estimator_input, const dynamics::StateVector_T<double>& zN_t_deviation) {
        dynamics::StateVector_T<double> zN_trim = dynamics::unpack_state_T(estimator_input.trim_sol.state);
        dynamics::StateVector_T<double> zN_t = zN_t_deviation + zN_trim;

        dynamics::RigidBodyState zN_t_packed = estimator_input.yN_t;
        dynamics::EulerAngles eul_meas;
        eul_meas.set(estimator_input.yN_t.q);
        dynamics::EulerAngles eul_est{ Eigen::Vector3d(eul_meas.psi(), zN_t(7), zN_t(6)) };
        dynamics::OrientationQuaternion q_est;
        q_est.set(eul_est);

        zN_t_packed.v = dynamics::TranslationalVelocity{ zN_t.segment<3>(0) };
        zN_t_packed.w = dynamics::AngularVelocity{ zN_t.segment<3>(3) };
        zN_t_packed.q = q_est;
        return zN_t_packed;
    }

    EstimationOutput LinearKalmanFilter::step(const KalmanFilterInput& estimator_input) {
        dynamics::StateVector_T<double> yt_deviation = makemeasurement_deviation(estimator_input);
        actuators::ActuatorInputsVector_T<double> ut_1_deviation = make_input_deviation(estimator_input);

        if (!initialized) {
            state = KalmanState{ .z = yt_deviation, .P = params.P0 };
            initialized = true;
        }

        linearization::DiscretizedTrimLinearization lin_sol_k = linearization::discretize(estimator_input.lin_sol);
        KalmanState predicted_state = policy.predict(state, lin_sol_k.A, lin_sol_k.B, ut_1_deviation);
        state = policy.correct(predicted_state, yt_deviation);

        dynamics::RigidBodyState zN_t = pack_state_estimate(estimator_input, state.z);

        return { .zN_t = zN_t };
    }
}
