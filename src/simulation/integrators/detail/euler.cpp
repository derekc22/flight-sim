#include <Eigen/Dense>
#include "simulation/constants/public/scalars.hpp"
#include "simulation/integrators/private/detail/euler.hpp"
#include "simulation/integrators/public/detail/euler.hpp"
#include "simulation/dynamics/public/detail/derivatives.hpp"
#include "simulation/structural/public/data/types.hpp"
#include "simulation/transforms/public/detail/s3.hpp"
#include "simulation/util/public/trig.hpp"

namespace integrators {

	dynamics::Position trans_kin(
	    const dynamics::Position& pt,
	    const dynamics::TranslationalVelocity& vt,
	    const dynamics::TranslationalAcceleration& at,
	    double dt)
	{
		const Eigen::Vector3d pt1 = pt.data + vt.data * dt + 0.5 * at.data * (dt * dt);

		return {pt1};
	}

	dynamics::OrientationQuaternion quat_kin(
	    const dynamics::OrientationQuaternion& qIB_t,
	    const dynamics::AngularVelocity& wB_BI_t,
	    double dt)
	{
		const Eigen::Vector3d w = wB_BI_t.data;
		const double Omega = w.norm();
		const double half_theta = 0.5 * Omega * dt;

		Eigen::Quaterniond dq;
		dq.w() = util::cos(half_theta);

		if (Omega < constants::eps) {
			dq.vec() = -0.5 * w * dt;
		} else {
			dq.vec() = -w * (util::sin(half_theta) / Omega);
		}

		const Eigen::Quaterniond qIB_t1 = dq * qIB_t.data;
		return {transforms::normalize_and_canonicalize(qIB_t1)};
	}

	dynamics::TranslationalVelocity trans_kin_vel(
	    const dynamics::TranslationalVelocity& vt,
	    const dynamics::TranslationalAcceleration& at,
	    double dt)
	{
		const Eigen::Vector3d vt1 = vt.data + at.data * dt;

		return {vt1};
	}

	dynamics::RigidBodyState step_rigid_body(
	    const dynamics::RigidBodyState& Xt,
	    const dynamics::Mass& mass,
	    const dynamics::InertiaTensor& JB,
	    const dynamics::Wrench& WB_net_t,
	    double dt)
	{

		// ddtB_vB_BI_t is the body derivative of body-expressed velocity,
		// ddtI_vB_BI_t is the inertial derivative of body-expressed velocity,
		// and aI_BI_t = CBI_t * aB_BI_t.data gives the inertial derivative of inertial-expressed velocity for the inertial-expressed dynamics::position update
		// Where CBI maps body components to inertial components
		// and ddtB_vB_BI returns the body derivative of body-expressed velocity

		const dynamics::Force FB_net_t = WB_net_t.F;
		const dynamics::Moment MB_net_t = WB_net_t.M;

		// Translational dynamics in body coordinates
		const dynamics::TranslationalVelocity vB_BI_t1 = trans_dyn_vel(Xt.v, Xt.w, mass, FB_net_t, dt);
		const Eigen::Vector3d ddtB_vB_BI_t =
		    dynamics::ddtB_vB_BI(Xt.v, Xt.w, mass, FB_net_t).data; // produces a body derivative
		const Eigen::Vector3d ddtI_vB_BI_t =
		    dynamics::ddtB_to_ddtI(ddtB_vB_BI_t, Xt.v.data, Xt.w.data); // produces an inertial derivative
		const dynamics::TranslationalAcceleration aB_BI_t{
		    ddtI_vB_BI_t}; // since pI_BI_t1 and vI_BI_t are inertial, aB_BI_t needs to be an inertial derivative

		// Rotational dynamics in body coordinates
		const dynamics::AngularVelocity wB_BI_t1 = rot_dyn(Xt.w, JB, MB_net_t, dt);

		// Quaternion rotational kinematics
		const dynamics::OrientationQuaternion qIB_t1 = quat_kin(Xt.q, Xt.w, dt);

		// Convert body velocity/acceleration to inertial for translational kinematics update on pI_BI
		const Eigen::Quaterniond qBI_t = Xt.q.data.conjugate();
		const dynamics::TranslationalVelocity vI_BI_t{qBI_t * Xt.v.data};
		const dynamics::TranslationalAcceleration aI_BI_t{qBI_t * aB_BI_t.data};

		// Translational kinematics in inertial coordinates
		const dynamics::Position pI_BI_t1 = trans_kin(Xt.p, vI_BI_t, aI_BI_t, dt);

		return {.p = pI_BI_t1, .v = vB_BI_t1, .q = qIB_t1, .w = wB_BI_t1};
	}

} // namespace integrators
