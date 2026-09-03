#include <Eigen/Dense>
#include "simulation/constants/public/scalars.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/integrators/private/detail/euler.hpp"
#include "simulation/dynamics/public/detail/derivatives.hpp"
#include "simulation/util/public/linalg.hpp"
#include "simulation/util/public/trig.hpp"

namespace integrators {

	dynamics::EulerAngles eul_kin(
	    const dynamics::EulerAngles& eul_t,
	    const dynamics::EulerAngleRates& eul_dot_t,
	    double dt)
	{
		const Eigen::Vector3d eul_t1 = eul_t.data + eul_dot_t.data * dt;

		return {eul_t1};
	}

	dynamics::OrientationMatrix rot_kin(
	    const dynamics::OrientationMatrix& CIB_t,
	    const dynamics::AngularVelocity& wB_BI_t,
	    double dt)
	{
		// strap_down/poisson
		const double Omega = wB_BI_t.data.norm();
		Eigen::Matrix3d exp_term;

		if (Omega < constants::eps)
			exp_term = constants::I3;
		else {
			const Eigen::Vector3d w_hat = wB_BI_t.data / Omega;
			Eigen::Matrix3d w_hat_skew = util::hat(w_hat);
			exp_term = constants::I3 - util::sin(Omega * dt) * w_hat_skew +
			    (1 - util::cos(Omega * dt)) * w_hat_skew * w_hat_skew;
		}

		const Eigen::Matrix3d CIB_t1 = exp_term * CIB_t.data;
		return {CIB_t1};
	}

	dynamics::TranslationalVelocity trans_dyn_vel(
	    const dynamics::TranslationalVelocity& vB_BI_t,
	    const dynamics::AngularVelocity& wB_BI_t,
	    const dynamics::Mass& mass,
	    const dynamics::Force& FB_net_t,
	    double dt)
	{
		const dynamics::TranslationalAcceleration vB_BI_dot_t = ddtB_vB_BI(vB_BI_t, wB_BI_t, mass, FB_net_t);

		const Eigen::Vector3d vB_BI_t1 = vB_BI_t.data + vB_BI_dot_t.data * dt;
		return {vB_BI_t1};
	}

	dynamics::AngularVelocity rot_dyn(
	    const dynamics::AngularVelocity& wB_BI_t,
	    const dynamics::InertiaTensor& JB,
	    const dynamics::Moment& MB_net_t,
	    double dt)
	{
		const Eigen::Vector3d wB_BI_dot_t = ddtB_wB_BI(wB_BI_t, JB, MB_net_t);

		const Eigen::Vector3d wB_BI_t1 = wB_BI_t.data + wB_BI_dot_t * dt;
		return {wB_BI_t1};
	}

} // namespace integrators
