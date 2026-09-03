#include <Eigen/Dense>
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/estimation/public/data/helpers.hpp"

namespace estimation {

	dynamics::RigidBodyState make_kalman_state_estimate(
	    const dynamics::RigidBodyState& Yt,
	    const dynamics::StateVector_T<double>& zt)
	{
		dynamics::RigidBodyState Zt = Yt;

		dynamics::EulerAngles eul_meas;
		eul_meas.set(Yt.q);

		dynamics::EulerAngles eul_est{Eigen::Vector3d(eul_meas.psi(), zt(7), zt(6))};
		dynamics::OrientationQuaternion q_est;
		q_est.set(eul_est);

		Zt.v = dynamics::TranslationalVelocity{zt.segment<3>(0)};
		Zt.w = dynamics::AngularVelocity{zt.segment<3>(3)};
		Zt.q = q_est;

		return Zt;
	}

} // namespace estimation
