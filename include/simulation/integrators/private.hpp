#pragma once
#include "simulation/integrators/public.hpp"

namespace propulsion { template <typename T> struct PropulsorOmegaDot_T; } // forward declare

namespace integrators {

    dynamics::Wrench compute_rigid_body_net_wrench(const dynamics::RigidBodyState& XB_BI_t, RK4Model& model, const operating::OperatingConditions& conditions, const actuators::SurfaceActuatorInputs_T<double>& u_surface, const actuators::PropulsorActuatorInputs_T<double>& u_propulsor, const propulsion::PropulsorOmegaDot_T<double>& omega_dot);
    RigidBodyStateDot compute_rigid_body_state_dot(const dynamics::RigidBodyState& XB_BI_t, const dynamics::Mass& mass, const dynamics::InertiaTensor& JB, const dynamics::Wrench& WB_net_t);
    dynamics::RigidBodyState add_scaled_rigid_body_state_dot(const dynamics::RigidBodyState& X, const RigidBodyStateDot& X_dot, double scale);
    dynamics::RigidBodyState add_rk4_weighted_rigid_body_state_dot(const dynamics::RigidBodyState& X, const RigidBodyStateDot& k1, const RigidBodyStateDot& k2, const RigidBodyStateDot& k3, const RigidBodyStateDot& k4, double dt);

}
