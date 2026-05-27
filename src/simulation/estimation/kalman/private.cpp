#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/linearization/public.hpp"
#include "simulation/estimation/kalman/private.hpp"
#include "simulation/estimation/kalman/public.hpp"

namespace estimation {

    dynamics::StateVector_T<double> make_measurement_deviation(const KalmanFilterEstimatorInput& estimator_input) {
        return dynamics::unpack_rigid_body_state(estimator_input.yN_t) - dynamics::unpack_state_T(estimator_input.operating_point.state);
    }

    actuators::ActuatorInputsVector_T<double> make_input_deviation(const KalmanFilterEstimatorInput& estimator_input) {
        return actuators::unpack_actuator_inputs(estimator_input.u_surface_actual_prev, estimator_input.u_propulsor_actual_prev) - actuators::unpack_actuator_inputs_T(estimator_input.operating_point.input);
    }

    dynamics::RigidBodyState pack_state_estimate(const KalmanFilterEstimatorInput& estimator_input, const dynamics::StateVector_T<double>& zN_t_pred, bool nonlinear_bool) {
        dynamics::StateVector_T<double> zN_trim = dynamics::unpack_state_T(estimator_input.operating_point.state);

        dynamics::StateVector_T<double> zN_t = nonlinear_bool 
            ? zN_t_pred             // EKF predicts full state, so no addition necessary
            : zN_t_pred + zN_trim;  // LKF predicts deviation state, so trim state is added back

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

    KalmanFilterInput make_kalman_filter_input(const KalmanFilterEstimatorInput& estimator_input, bool nonlinear_bool) {
        dynamics::StateVector_T<double> yN_T = nonlinear_bool 
            ? dynamics::unpack_rigid_body_state(estimator_input.yN_t) 
            : make_measurement_deviation(estimator_input);

        actuators::ActuatorInputsVector_T<double> ut_1 = nonlinear_bool
            ? actuators::unpack_actuator_inputs(estimator_input.u_surface_actual_prev, estimator_input.u_propulsor_actual_prev)
            : make_input_deviation(estimator_input);

        linearization::DiscretizedLocalLinearization lin_sol_k = linearization::discretize(estimator_input.lin_sol);

        return { 
            .A = lin_sol_k.A,
            .B = lin_sol_k.B,
            .C = lin_sol_k.C,
            .yN_t = yN_T,
            .ut_1 = ut_1,
            .conditions = estimator_input.conditions,
            .aircraft = estimator_input.aircraft,
            .nonlinear_bool = nonlinear_bool
        };
        
    }

}