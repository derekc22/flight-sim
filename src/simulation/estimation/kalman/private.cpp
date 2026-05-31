#include <Eigen/Dense>
#include "simulation/dynamics/public.hpp"
#include "simulation/estimation/kalman/private.hpp"

namespace estimation {

    dynamics::RigidBodyState make_kalman_state_estimate(const dynamics::RigidBodyState& yN_t, const dynamics::StateVector_T<double>& zN_t) {
        dynamics::RigidBodyState zN_t_rbs = yN_t;

        dynamics::EulerAngles eul_meas;
        eul_meas.set(yN_t.q);
        dynamics::EulerAngles eul_est{ Eigen::Vector3d(eul_meas.psi(), zN_t(7), zN_t(6)) };
        dynamics::OrientationQuaternion q_est;
        q_est.set(eul_est);

        zN_t_rbs.v = dynamics::TranslationalVelocity{ zN_t.segment<3>(0) };
        zN_t_rbs.w = dynamics::AngularVelocity{ zN_t.segment<3>(3) };
        zN_t_rbs.q = q_est;

        return zN_t_rbs;
    }

}