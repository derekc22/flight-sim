#include "simulation/dynamics/public/detail/state.hpp"

#include "simulation/frames/public/detail/kinematics.hpp"

namespace dynamics
{

	RigidBodyState get_rigid_body_state(
		const frames::Frame& F)
	{
		const frames::FrameView fv = F.view();
		return {.p = fv.H->p(), .v = *fv.v, .q = *fv.q, .w = *fv.w};
	}

	RigidBodyState compute_rigid_body_state(
		const frames::Frame& F,
		const frames::Frame& R)
	{
		dynamics::HomogeneousTransformationMatrix HRF = frames::H_from_R(F, R);
		dynamics::OrientationQuaternion qRF;
		qRF.set(HRF.C());

		auto [vF_FR, wF_FR] = frames::vel_from_R(F, R);

		return {.p = HRF.p(), .v = vF_FR, .q = qRF, .w = wF_FR};
	}

	dynamics::RigidBodyState invert_rigid_body_state(
		const dynamics::RigidBodyState& X_BA)
	{
		dynamics::OrientationMatrix CAB;
		CAB.set(X_BA.q);

		dynamics::OrientationMatrix CBA{CAB.data.transpose()};
		dynamics::OrientationQuaternion qBA;
		qBA.set(CBA);

		dynamics::Position pB_AB{-CAB.data * X_BA.p.data};

		return {
			.p = pB_AB,
			.v = dynamics::TranslationalVelocity{-CBA.data * (X_BA.v.data + X_BA.w.data.cross(pB_AB.data))},
			.q = qBA,
			.w = dynamics::AngularVelocity{-CBA.data * X_BA.w.data}
		};
	}

	dynamics::RigidBodyState compose_rigid_body_state(
		const dynamics::RigidBodyState& X_BA,
		const dynamics::RigidBodyState& X_AR)
	{
		dynamics::OrientationMatrix CRA;
		CRA.set(X_AR.q);

		dynamics::OrientationMatrix CAB;
		CAB.set(X_BA.q);

		dynamics::OrientationMatrix CRB{CAB.data * CRA.data};
		dynamics::OrientationQuaternion qRB;
		qRB.set(CRB);

		return {
			.p = dynamics::Position{X_AR.p.data + CRA.data.transpose() * X_BA.p.data},
			// vBR = vAR + vBA + wAR x pBA
			.v =
				dynamics::TranslationalVelocity{
					X_BA.v.data + CAB.data * (X_AR.v.data + X_AR.w.data.cross(X_BA.p.data))
				},
			.q = qRB,
			.w = dynamics::AngularVelocity{X_BA.w.data + CAB.data * X_AR.w.data}
		};
	}

	dynamics::RigidBodyState rebase_cg_state(
		const dynamics::RigidBodyState& X_GN,
		const dynamics::Position& pB_GB,
		const frames::Frame& CGFrame)
	{
		dynamics::RigidBodyState X_GB = dynamics::get_rigid_body_state(CGFrame);
		X_GB.p = pB_GB;
		dynamics::RigidBodyState X_BG = dynamics::invert_rigid_body_state(X_GB);
		return dynamics::compose_rigid_body_state(X_BG, X_GN);
	}

} // namespace dynamics
