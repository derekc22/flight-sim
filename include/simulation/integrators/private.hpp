#pragma once
#include <tuple>
#include "simulation/dynamics/public.hpp"
#include "simulation/integrators/public.hpp"
#include "simulation/propulsion/public.hpp"

namespace integrators {

    dynamics::EulerAngles eul_kin(const dynamics::EulerAngles& eul_t, const dynamics::EulerAngleRates& eul_dot_t, double dt);

    dynamics::OrientationMatrix rot_kin(const dynamics::OrientationMatrix& CIB_t, const dynamics::AngularVelocity& wB_BI_t, double dt);

    dynamics::TranslationalVelocity trans_dyn_vel(const dynamics::TranslationalVelocity& vB_BI_t, const dynamics::AngularVelocity& wB_BI_t, const dynamics::Mass& mass, const dynamics::Force& FB_net_t, double dt);

    dynamics::AngularVelocity rot_dyn(const dynamics::AngularVelocity& wB_BI_t, const dynamics::InertiaTensor& JB, const dynamics::Moment& MB_net_t, double dt);

    std::tuple<dynamics::Wrench, dynamics::Wrench, dynamics::Wrench> compute_rigid_body_net_wrench(const dynamics::RigidBodyState& Xt, RK4Model& model, const operating::OperatingConditions& conditions, const actuators::SurfaceActuatorInputs_T<double>& u_surface, const actuators::PropulsorActuatorInputs_T<double>& u_propulsor, const propulsion::PropulsorOmegaDot_T<double>& omega_dot);

    RigidBodyStateDot compute_rigid_body_state_dot(const dynamics::RigidBodyState& Xt, const dynamics::Mass& mass, const dynamics::InertiaTensor& JB, const dynamics::Wrench& WB_net_t);

    dynamics::RigidBodyState add_scaled_rigid_body_state_dot(const dynamics::RigidBodyState& X, const RigidBodyStateDot& X_dot, double scale);

    dynamics::RigidBodyState add_rk4_weighted_rigid_body_state_dot(const dynamics::RigidBodyState& X, const RigidBodyStateDot& k1, const RigidBodyStateDot& k2, const RigidBodyStateDot& k3, const RigidBodyStateDot& k4, double dt);

}
