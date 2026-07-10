#include <Eigen/Dense>
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/private.hpp"
#include "simulation/integrators/private.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/propulsion/public.hpp"
#include "simulation/structural/public.hpp"
#include "simulation/transforms/public.hpp"

namespace integrators {

    dynamics::Position trans_kin(const dynamics::Position& pt, const dynamics::TranslationalVelocity& vt, const dynamics::TranslationalAcceleration& at, double dt) {
        const Eigen::Vector3d pt1 = pt.data + vt.data * dt + 0.5 * at.data * (dt * dt);

        return { pt1 };
    }

    dynamics::OrientationQuaternion quat_kin(const dynamics::OrientationQuaternion& qIB_t, const dynamics::AngularVelocity& wB_BI_t, double dt) {
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
        return { transforms::normalize_and_canonicalize(qIB_t1) };
    }

    dynamics::TranslationalVelocity trans_kin_vel(const dynamics::TranslationalVelocity& vt, const dynamics::TranslationalAcceleration& at, double dt) {
        const Eigen::Vector3d vt1 = vt.data + at.data * dt;

        return { vt1 };
    }

    dynamics::RigidBodyState step_rigid_body(const dynamics::RigidBodyState& Xt, const dynamics::Mass& mass, const dynamics::InertiaTensor& JB, const dynamics::Wrench& WB_net_t, double dt) {

        // ddtB_vB_BI_t is the body derivative of body-expressed velocity, 
        // ddtI_vB_BI_t is the inertial derivative of body-expressed velocity, 
        // and aI_BI_t = CBI_t * aB_BI_t.data gives the inertial derivative of inertial-expressed velocity for the inertial-expressed dynamics::position update
        // Where CBI maps body components to inertial components 
        // and ddtB_vB_BI returns the body derivative of body-expressed velocity

        const dynamics::Force FB_net_t = WB_net_t.F;
        const dynamics::Moment MB_net_t = WB_net_t.M;

        const Eigen::Matrix3d CIB_t = transforms::quat_to_rot(Xt.q.data);
        const Eigen::Matrix3d CBI_t = CIB_t.transpose();

        // Translational dynamics in body coordinates
        const dynamics::TranslationalVelocity vB_BI_t1 = trans_dyn_vel(Xt.v, Xt.w, mass, FB_net_t, dt);
        const Eigen::Vector3d ddtB_vB_BI_t = dynamics::ddtB_vB_BI(Xt.v, Xt.w, mass, FB_net_t).data;           // produces a body derivative
        const Eigen::Vector3d ddtI_vB_BI_t = dynamics::ddtB_to_ddtI(ddtB_vB_BI_t, Xt.v.data, Xt.w.data);      // produces an inertial derivative
        const dynamics::TranslationalAcceleration aB_BI_t { ddtI_vB_BI_t }; // since pI_BI_t1 and vI_BI_t are inertial, aB_BI_t needs to be an inertial derivative

        // Rotational dynamics in body coordinates
        const dynamics::AngularVelocity wB_BI_t1 = rot_dyn(Xt.w, JB, MB_net_t, dt);

        // Quaternion rotational kinematics
        const dynamics::OrientationQuaternion qIB_t1 = quat_kin(Xt.q, Xt.w, dt);

        // Convert body velocity/acceleration to inertial for translational kinematics update on pI_BI
        const dynamics::TranslationalVelocity vI_BI_t { CBI_t * Xt.v.data };
        const dynamics::TranslationalAcceleration aI_BI_t { CBI_t * aB_BI_t.data };

        // Translational kinematics in inertial coordinates
        const dynamics::Position pI_BI_t1 = trans_kin(Xt.p, vI_BI_t, aI_BI_t, dt);

        return { .p = pI_BI_t1, .v = vB_BI_t1, .q = qIB_t1, .w = wB_BI_t1 };
    }

    RK4Output step_rigid_body_rk4(const dynamics::RigidBodyState& Xt, RK4Model& model, const operating::OperatingConditions& conditions, const actuators::SurfaceActuatorInputs_T<double>& u_surface, const actuators::PropulsorActuatorInputs_T<double>& u_propulsor, double dt) {
        const propulsion::PropellerOmegaState_T<double> front_propulsor_omega_state = propulsion::compute_propeller_omega_state_T<double>(
            model.propulsor_actuators.front_propulsor,
            u_propulsor.front_propulsor_cmd,
            conditions.atm.rho,
            dt
        );
        const propulsion::PropellerOmegaState_T<double> left_propulsor_omega_state = propulsion::compute_propeller_omega_state_T<double>(
            model.propulsor_actuators.left_propulsor,
            u_propulsor.left_propulsor_cmd,
            conditions.atm.rho,
            dt
        );
        const propulsion::PropellerOmegaState_T<double> right_propulsor_omega_state = propulsion::compute_propeller_omega_state_T<double>(
            model.propulsor_actuators.right_propulsor,
            u_propulsor.right_propulsor_cmd,
            conditions.atm.rho,
            dt
        );

        const propulsion::PropulsorOmegaDot_T<double> omega_dot{
            .front_propulsor = front_propulsor_omega_state.omega_dot,
            .left_propulsor = left_propulsor_omega_state.omega_dot,
            .right_propulsor = right_propulsor_omega_state.omega_dot
        };

        auto [WB_net_1, WB_aerodynamic_1, WB_propulsive_1] = compute_rigid_body_net_wrench(Xt, model, conditions, u_surface, u_propulsor, omega_dot);
        const RigidBodyStateDot k1 = compute_rigid_body_state_dot(Xt, model.structural.mass, model.structural.JB, WB_net_1);

        const dynamics::RigidBodyState X2 = add_scaled_rigid_body_state_dot(Xt, k1, 0.5 * dt);
        auto [WB_net_2, WB_aerodynamic_2, WB_propulsive_2] = compute_rigid_body_net_wrench(X2, model, conditions, u_surface, u_propulsor, omega_dot);
        const RigidBodyStateDot k2 = compute_rigid_body_state_dot(X2, model.structural.mass, model.structural.JB, WB_net_2);

        const dynamics::RigidBodyState X3 = add_scaled_rigid_body_state_dot(Xt, k2, 0.5 * dt);
        auto [WB_net_3, WB_aerodynamic_3, WB_propulsive_3] = compute_rigid_body_net_wrench(X3, model, conditions, u_surface, u_propulsor, omega_dot);
        const RigidBodyStateDot k3 = compute_rigid_body_state_dot(X3, model.structural.mass, model.structural.JB, WB_net_3);

        const dynamics::RigidBodyState X4 = add_scaled_rigid_body_state_dot(Xt, k3, dt);
        auto [WB_net_4, WB_aerodynamic_4, WB_propulsive_4] = compute_rigid_body_net_wrench(X4, model, conditions, u_surface, u_propulsor, omega_dot);
        const RigidBodyStateDot k4 = compute_rigid_body_state_dot(X4, model.structural.mass, model.structural.JB, WB_net_4);

        const dynamics::RigidBodyState Xt1 = add_rk4_weighted_rigid_body_state_dot(Xt, k1, k2, k3, k4, dt);

        propulsion::commit_propellers_omega_state(model.propulsor_actuators, front_propulsor_omega_state, left_propulsor_omega_state, right_propulsor_omega_state);

        return { 
            .Xt1 = Xt1, 
            .WB_net = WB_net_1, 
            .WB_aerodynamic = WB_aerodynamic_1, 
            .WB_propulsive = WB_propulsive_1 
        };
    }

}
