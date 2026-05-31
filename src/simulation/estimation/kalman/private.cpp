#include <Eigen/Dense>
#include "simulation/dynamics/public.hpp"
#include "simulation/estimation/kalman/private.hpp"

namespace estimation {

    dynamics::RigidBodyState make_kalman_state_estimate(const dynamics::RigidBodyState& Yt, const dynamics::StateVector_T<double>& zt_vec) {
        dynamics::RigidBodyState Zt = Yt;

        dynamics::EulerAngles eul_meas;
        eul_meas.set(Yt.q);
        dynamics::EulerAngles eul_est{ Eigen::Vector3d(eul_meas.psi(), zt_vec(7), zt_vec(6)) };
        dynamics::OrientationQuaternion q_est;
        q_est.set(eul_est);

        Zt.v = dynamics::TranslationalVelocity{ zt_vec.segment<3>(0) };
        Zt.w = dynamics::AngularVelocity{ zt_vec.segment<3>(3) };
        Zt.q = q_est;

        return Zt;
    }

}
