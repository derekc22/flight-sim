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

    dynamics::EulerAngles eul_kin(const dynamics::EulerAngles& eul_t, const dynamics::EulerAngleRates& eul_dot_t, double dt) {
        const Eigen::Vector3d eul_t1 = eul_t.data + eul_dot_t.data * dt;

        return { eul_t1 };
    }

    dynamics::OrientationMatrix rot_kin(const dynamics::OrientationMatrix& CIB_t, const dynamics::AngularVelocity& wB_BI_t, double dt) {
        // strap_down/poisson
        const double Omega = wB_BI_t.data.norm();
        Eigen::Matrix3d exp_term;

        if (Omega < constants::eps) exp_term = constants::I3; 
        else {
            const Eigen::Vector3d w_hat = wB_BI_t.data/Omega;
            Eigen::Matrix3d w_hat_skew = util::hat(w_hat);
            exp_term = constants::I3 - util::sin(Omega * dt) * w_hat_skew + (1 - util::cos(Omega * dt)) * w_hat_skew * w_hat_skew;
        }

        const Eigen::Matrix3d CIB_t1 = exp_term * CIB_t.data;
        return { CIB_t1 };
    }

    dynamics::TranslationalVelocity trans_dyn_vel(const dynamics::TranslationalVelocity& vB_BI_t, const dynamics::AngularVelocity& wB_BI_t, const dynamics::Mass& mass, const dynamics::Force& FB_net_t, double dt) {
        const dynamics::TranslationalAcceleration vB_BI_dot_t = ddtB_vB_BI(vB_BI_t, wB_BI_t, mass, FB_net_t);

        const Eigen::Vector3d vB_BI_t1 = vB_BI_t.data + vB_BI_dot_t.data * dt;
        return { vB_BI_t1 };
    }

    dynamics::AngularVelocity rot_dyn(const dynamics::AngularVelocity& wB_BI_t, const dynamics::InertiaTensor& JB, const dynamics::Moment& MB_net_t, double dt) {
        const Eigen::Vector3d wB_BI_dot_t = ddtB_wB_BI(wB_BI_t, JB, MB_net_t);

        const Eigen::Vector3d wB_BI_t1 = wB_BI_t.data + wB_BI_dot_t * dt;
        return { wB_BI_t1 };
    }

    dynamics::WrenchSet compute_net_wrench(const dynamics::RigidBodyState& Xt, RK4Model& model, const operating::OperatingConditions& conditions, const actuators::ActuatorInputs_T<double>& u, const propulsion::PropellerOmegaDotSet_T<double>& propeller_omega_dot_set) {
        const dynamics::Twist_T<double> twist{
            .v = Xt.v.data,
            .w = Xt.w.data
        };

        const atmospheric::Wind windB{ Xt.q.data * conditions.windI.data };
        const Eigen::Vector3d gB = geography::gB(Xt.q).data;
        const dynamics::WrenchSet_T<double> wrench = compute_wrench_set_T<double>(model, twist, conditions.atm, u, propeller_omega_dot_set, windB, gB);

        return {
            .aerodynamic = { 
                .F = dynamics::Force{ wrench.aerodynamic.F },
                .M = dynamics::Moment{ wrench.aerodynamic.M }
            },
            .propulsive = { 
                .F = dynamics::Force{ wrench.propulsive.F },
                .M = dynamics::Moment{ wrench.propulsive.M }
            },
            .net = { 
                .F = dynamics::Force{ wrench.net.F },
                .M = dynamics::Moment{ wrench.net.M }
            }
        };
    }

    dynamics::RigidBodyStateDot compute_rigid_body_state_dot(const dynamics::RigidBodyState& Xt, const dynamics::Mass& mass, const dynamics::InertiaTensor& JB, const dynamics::Wrench& WB_net_t) {
        const dynamics::Force FB_net_t = WB_net_t.F;
        const dynamics::Moment MB_net_t = WB_net_t.M;

        return {
            .p_dot = dynamics::TranslationalVelocity{ Xt.q.data.conjugate() * Xt.v.data },
            .v_dot = dynamics::ddtB_vB_BI(Xt.v, Xt.w, mass, FB_net_t),
            .w_dot = dynamics::AngularAcceleration{ dynamics::ddtB_wB_BI(Xt.w, JB, MB_net_t) }
        };
    }

    dynamics::RigidBodyState add_scaled_rigid_body_state_dot(const dynamics::RigidBodyState& X, const dynamics::RigidBodyStateDot& X_dot, double scale) {
        const dynamics::AngularVelocity w{ X.w.data + X_dot.w_dot.data * scale };

        return {
            .p = dynamics::Position{ X.p.data + X_dot.p_dot.data * scale },
            .v = dynamics::TranslationalVelocity{ X.v.data + X_dot.v_dot.data * scale },
            .q = quat_kin(X.q, w, scale),
            .w = w
        };
    }

    dynamics::RigidBodyState add_rk4_weighted_rigid_body_state_dot(const dynamics::RigidBodyState& X, const dynamics::RigidBodyStateDot& k1, const dynamics::RigidBodyStateDot& k2, const dynamics::RigidBodyStateDot& k3, const dynamics::RigidBodyStateDot& k4, double dt) {
        dynamics::RigidBodyStateDot X_dot{
            .p_dot = dynamics::TranslationalVelocity{ (
                k1.p_dot.data + 
                2.0 * k2.p_dot.data + 
                2.0 * k3.p_dot.data + 
                k4.p_dot.data) / 6.0 
            },
            .v_dot = dynamics::TranslationalAcceleration{ (
                k1.v_dot.data + 
                2.0 * k2.v_dot.data + 
                2.0 * k3.v_dot.data + 
                k4.v_dot.data) / 6.0 
            },
            .w_dot = dynamics::AngularAcceleration{ (
                k1.w_dot.data + 
                2.0 * k2.w_dot.data + 
                2.0 * k3.w_dot.data + 
                k4.w_dot.data) / 6.0 
            }
        };

        const dynamics::AngularVelocity w1{ X.w.data };
        const dynamics::AngularVelocity w2{ X.w.data + k1.w_dot.data * 0.5 * dt };
        const dynamics::AngularVelocity w3{ X.w.data + k2.w_dot.data * 0.5 * dt };
        const dynamics::AngularVelocity w4{ X.w.data + k3.w_dot.data * dt };
        const dynamics::AngularVelocity w_avg{ (w1.data + 2.0 * w2.data + 2.0 * w3.data + w4.data) / 6.0 };

        return {
            .p = dynamics::Position{ X.p.data + X_dot.p_dot.data * dt },
            .v = dynamics::TranslationalVelocity{ X.v.data + X_dot.v_dot.data * dt },
            .q = quat_kin(X.q, w_avg, dt),
            .w = dynamics::AngularVelocity{ X.w.data + X_dot.w_dot.data * dt }
        };
    }

}
