#include <Eigen/Dense>
#include <tuple>
#include "simulation/actuators/public.hpp"
#include "simulation/dynamics/private.hpp"
#include "simulation/integrators/private.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/propulsion/public.hpp"
#include "simulation/structural/public.hpp"
#include "simulation/transforms/public.hpp"

namespace integrators {

    dynamics::RigidBodyState step_rigid_body(const dynamics::RigidBodyState& XB_BI_t, const dynamics::Mass& mass, const dynamics::InertiaTensor& JB, const dynamics::Wrench& WB_net_t, double dt) {

        // ddtB_vB_BI_t is the body derivative of body-expressed velocity, 
        // ddtI_vB_BI_t is the inertial derivative of body-expressed velocity, 
        // and aI_BI_t = CBI_t * aB_BI_t.data gives the inertial derivative of inertial-expressed velocity for the inertial-expressed position update
        // Where CBI maps body components to inertial components 
        // and ddtB_vB_BI returns the body derivative of body-expressed velocity

        const dynamics::Force FB_net_t = WB_net_t.F;
        const dynamics::Moment MB_net_t = WB_net_t.M;

        const Eigen::Matrix3d CIB_t = transforms::quat_to_rot(XB_BI_t.q.data);
        const Eigen::Matrix3d CBI_t = CIB_t.transpose();

        // Translational dynamics in body coordinates
        const dynamics::TranslationalVelocity vB_BI_t1 = dynamics::trans_dyn_vel(XB_BI_t.v, XB_BI_t.w, mass, FB_net_t, dt);
        const Eigen::Vector3d ddtB_vB_BI_t = dynamics::ddtB_vB_BI(XB_BI_t.v, XB_BI_t.w, mass, FB_net_t).data;           // produces a body derivative
        const Eigen::Vector3d ddtI_vB_BI_t = dynamics::ddtB_to_ddtI(ddtB_vB_BI_t, XB_BI_t.v.data, XB_BI_t.w.data);      // produces an inertial derivative
        const dynamics::TranslationalAcceleration aB_BI_t { ddtI_vB_BI_t }; // since pI_BI_t1 and vI_BI_t are inertial, aB_BI_t needs to be an inertial derivative

        // Rotational dynamics in body coordinates
        const dynamics::AngularVelocity wB_BI_t1 = dynamics::rot_dyn(XB_BI_t.w, JB, MB_net_t, dt);

        // Quaternion rotational kinematics
        const dynamics::OrientationQuaternion qIB_t1 = dynamics::quat_kin(XB_BI_t.q, XB_BI_t.w, dt);

        // Convert body velocity/acceleration to inertial for translational kinematics update on pI_BI
        const dynamics::TranslationalVelocity vI_BI_t { CBI_t * XB_BI_t.v.data };
        const dynamics::TranslationalAcceleration aI_BI_t { CBI_t * aB_BI_t.data };

        // Translational kinematics in inertial coordinates
        const dynamics::Position pI_BI_t1 = dynamics::trans_kin(XB_BI_t.p, vI_BI_t, aI_BI_t, dt);

        return { .p = pI_BI_t1, .v = vB_BI_t1, .q = qIB_t1, .w = wB_BI_t1 };
    }

    std::tuple<dynamics::RigidBodyState, dynamics::Wrench> step_rigid_body_rk4(const dynamics::RigidBodyState& XB_BI_t, RK4Model& model, const operating::OperatingConditions& conditions, const actuators::SurfaceActuatorInputs_T<double>& u_surface, const actuators::PropulsorActuatorInputs_T<double>& u_propulsor, double dt) {
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

        const dynamics::Wrench WB_net_1 = compute_rigid_body_net_wrench(XB_BI_t, model, conditions, u_surface, u_propulsor, omega_dot);
        const RigidBodyStateDot k1 = compute_rigid_body_state_dot(XB_BI_t, model.structural.mass, model.structural.JB, WB_net_1);

        const dynamics::RigidBodyState X2 = add_scaled_rigid_body_state_dot(XB_BI_t, k1, 0.5 * dt);
        const dynamics::Wrench WB_net_2 = compute_rigid_body_net_wrench(X2, model, conditions, u_surface, u_propulsor, omega_dot);
        const RigidBodyStateDot k2 = compute_rigid_body_state_dot(X2, model.structural.mass, model.structural.JB, WB_net_2);

        const dynamics::RigidBodyState X3 = add_scaled_rigid_body_state_dot(XB_BI_t, k2, 0.5 * dt);
        const dynamics::Wrench WB_net_3 = compute_rigid_body_net_wrench(X3, model, conditions, u_surface, u_propulsor, omega_dot);
        const RigidBodyStateDot k3 = compute_rigid_body_state_dot(X3, model.structural.mass, model.structural.JB, WB_net_3);

        const dynamics::RigidBodyState X4 = add_scaled_rigid_body_state_dot(XB_BI_t, k3, dt);
        const dynamics::Wrench WB_net_4 = compute_rigid_body_net_wrench(X4, model, conditions, u_surface, u_propulsor, omega_dot);
        const RigidBodyStateDot k4 = compute_rigid_body_state_dot(X4, model.structural.mass, model.structural.JB, WB_net_4);

        const dynamics::RigidBodyState XB_BI_t1 = add_rk4_weighted_rigid_body_state_dot(XB_BI_t, k1, k2, k3, k4, dt);

        propulsion::commit_propellers_omega_state(model.propulsor_actuators, front_propulsor_omega_state, left_propulsor_omega_state, right_propulsor_omega_state);

        return { XB_BI_t1, WB_net_1 };
    }

}
