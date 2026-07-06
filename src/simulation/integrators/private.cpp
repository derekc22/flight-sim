#include <Eigen/Dense>
#include "simulation/actuators/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/atmospheric/public.hpp"
#include "simulation/dynamics/private.hpp"
#include "simulation/geography/public.hpp"
#include "simulation/integrators/private.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/propulsion/public.hpp"
#include "simulation/structural/public.hpp"
#include "simulation/transforms/public.hpp"

namespace integrators {

    dynamics::Wrench compute_rigid_body_net_wrench(const dynamics::RigidBodyState& XB_BI_t, RK4Model& model, const operating::OperatingConditions& conditions, const actuators::SurfaceActuatorInputs_T<double>& u_surface, const actuators::PropulsorActuatorInputs_T<double>& u_propulsor, const propulsion::PropulsorOmegaDot_T<double>& omega_dot) {
        const dynamics::Twist_T<double> twist{
            .v = XB_BI_t.v.data,
            .w = XB_BI_t.w.data
        };

        const atmospheric::Wind windB{ transforms::quat_to_rot(XB_BI_t.q.data) * conditions.windI.data };
        const dynamics::Wrench_T<double> aero_wrench = aerodynamics::step_aero_forces_moments_T<double>(model.aerodynamic, twist, conditions.atm, u_surface, windB);

        const dynamics::Wrench_T<double> prop_wrench = propulsion::step_propulsive_forces_moments_T<double>(model.propulsor_actuators, twist, conditions.atm, u_propulsor, omega_dot);

        const Eigen::Vector3d FB_g = model.structural.mass.data * geography::gB(XB_BI_t.q).data;

        return {
            .F = dynamics::Force{ FB_g + aero_wrench.F + prop_wrench.F },
            .M = dynamics::Moment{ aero_wrench.M + prop_wrench.M }
        };
    }

    RigidBodyStateDot compute_rigid_body_state_dot(const dynamics::RigidBodyState& XB_BI_t, const dynamics::Mass& mass, const dynamics::InertiaTensor& JB, const dynamics::Wrench& WB_net_t) {
        const dynamics::Force FB_net_t = WB_net_t.F;
        const dynamics::Moment MB_net_t = WB_net_t.M;

        const Eigen::Matrix3d CIB_t = transforms::quat_to_rot(XB_BI_t.q.data);
        const Eigen::Matrix3d CBI_t = CIB_t.transpose();

        return {
            .p_dot = dynamics::TranslationalVelocity{ CBI_t * XB_BI_t.v.data },
            .v_dot = dynamics::ddtB_vB_BI(XB_BI_t.v, XB_BI_t.w, mass, FB_net_t),
            .q_dot = dynamics::quat_kin_vel(XB_BI_t.q, XB_BI_t.w),
            .w_dot = dynamics::AngularAcceleration{ dynamics::ddtB_wB_BI(XB_BI_t.w, JB, MB_net_t) }
        };
    }

    dynamics::RigidBodyState add_scaled_rigid_body_state_dot(const dynamics::RigidBodyState& X, const RigidBodyStateDot& X_dot, double scale) {
        Eigen::Quaterniond q = X.q.data;
        q.coeffs() += X_dot.q_dot.data.coeffs() * scale;

        return {
            .p = dynamics::Position{ X.p.data + X_dot.p_dot.data * scale },
            .v = dynamics::TranslationalVelocity{ X.v.data + X_dot.v_dot.data * scale },
            .q = dynamics::OrientationQuaternion{ transforms::normalize_and_canonicalize(q) },
            .w = dynamics::AngularVelocity{ X.w.data + X_dot.w_dot.data * scale }
        };
    }

    dynamics::RigidBodyState add_rk4_weighted_rigid_body_state_dot(const dynamics::RigidBodyState& X, const RigidBodyStateDot& k1, const RigidBodyStateDot& k2, const RigidBodyStateDot& k3, const RigidBodyStateDot& k4, double dt) {
        RigidBodyStateDot X_dot{
            .p_dot = dynamics::TranslationalVelocity{ (k1.p_dot.data + 2.0 * k2.p_dot.data + 2.0 * k3.p_dot.data + k4.p_dot.data) / 6.0 },
            .v_dot = dynamics::TranslationalAcceleration{ (k1.v_dot.data + 2.0 * k2.v_dot.data + 2.0 * k3.v_dot.data + k4.v_dot.data) / 6.0 },
            .q_dot = dynamics::OrientationQuaternionRate{},
            .w_dot = dynamics::AngularAcceleration{ (k1.w_dot.data + 2.0 * k2.w_dot.data + 2.0 * k3.w_dot.data + k4.w_dot.data) / 6.0 }
        };
        X_dot.q_dot.data.coeffs() = (k1.q_dot.data.coeffs() + 2.0 * k2.q_dot.data.coeffs() + 2.0 * k3.q_dot.data.coeffs() + k4.q_dot.data.coeffs()) / 6.0;
        return add_scaled_rigid_body_state_dot(X, X_dot, dt);
    }

}
