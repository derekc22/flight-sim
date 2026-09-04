#include "simulation/integrators/private/detail/rk4.hpp"

#include "simulation/dynamics/public/detail/derivatives.hpp"
#include "simulation/integrators/public/detail/euler.hpp"

#include <Eigen/Dense>

namespace integrators
{

	dynamics::RigidBodyStateDot compute_rigid_body_state_dot(
		const dynamics::RigidBodyState& Xt,
		const dynamics::Mass& mass,
		const dynamics::InertiaTensor& JB,
		const dynamics::Wrench& WB_net_t)
	{
		const dynamics::Force FB_net_t = WB_net_t.F;
		const dynamics::Moment MB_net_t = WB_net_t.M;

		return {
			.p_dot = dynamics::TranslationalVelocity{Xt.q.data.conjugate() * Xt.v.data},
			.v_dot = dynamics::ddtB_vB_BI(Xt.v, Xt.w, mass, FB_net_t),
			.w_dot = dynamics::AngularAcceleration{dynamics::ddtB_wB_BI(Xt.w, JB, MB_net_t)}
		};
	}

	dynamics::RigidBodyState add_scaled_rigid_body_state_dot(
		const dynamics::RigidBodyState& X,
		const dynamics::RigidBodyStateDot& X_dot,
		double scale)
	{
		const dynamics::AngularVelocity w{X.w.data + X_dot.w_dot.data * scale};

		return {
			.p = dynamics::Position{X.p.data + X_dot.p_dot.data * scale},
			.v = dynamics::TranslationalVelocity{X.v.data + X_dot.v_dot.data * scale},
			.q = quat_kin(X.q, w, scale),
			.w = w
		};
	}

	dynamics::RigidBodyState add_rk4_weighted_rigid_body_state_dot(
		const dynamics::RigidBodyState& X,
		const dynamics::RigidBodyStateDot& k1,
		const dynamics::RigidBodyStateDot& k2,
		const dynamics::RigidBodyStateDot& k3,
		const dynamics::RigidBodyStateDot& k4,
		double dt)
	{
		dynamics::RigidBodyStateDot X_dot{
			.p_dot =
				dynamics::TranslationalVelocity{
					(k1.p_dot.data + 2.0 * k2.p_dot.data + 2.0 * k3.p_dot.data + k4.p_dot.data) / 6.0
				},
			.v_dot =
				dynamics::TranslationalAcceleration{
					(k1.v_dot.data + 2.0 * k2.v_dot.data + 2.0 * k3.v_dot.data + k4.v_dot.data) / 6.0
				},
			.w_dot = dynamics::AngularAcceleration{
				(k1.w_dot.data + 2.0 * k2.w_dot.data + 2.0 * k3.w_dot.data + k4.w_dot.data) / 6.0
			}
		};

		const dynamics::AngularVelocity w1{X.w.data};
		const dynamics::AngularVelocity w2{X.w.data + k1.w_dot.data * 0.5 * dt};
		const dynamics::AngularVelocity w3{X.w.data + k2.w_dot.data * 0.5 * dt};
		const dynamics::AngularVelocity w4{X.w.data + k3.w_dot.data * dt};
		const dynamics::AngularVelocity w_avg{(w1.data + 2.0 * w2.data + 2.0 * w3.data + w4.data) / 6.0};

		return {
			.p = dynamics::Position{X.p.data + X_dot.p_dot.data * dt},
			.v = dynamics::TranslationalVelocity{X.v.data + X_dot.v_dot.data * dt},
			.q = quat_kin(X.q, w_avg, dt),
			.w = dynamics::AngularVelocity{X.w.data + X_dot.w_dot.data * dt}
		};
	}

} // namespace integrators
