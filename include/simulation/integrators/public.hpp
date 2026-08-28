#pragma once
#include "simulation/atmospheric/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/structural/public.hpp"
#include "simulation/aerodynamics/public.hpp"
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/operating/public.hpp"
#include "simulation/propulsion/public.hpp"

namespace integrators {

    struct RK4Model {
        const structural::StructuralState& struc_t;
        const aerodynamics::AerodynamicsManager& aerodynamic;
        actuators::PropulsorActuators& propulsor_actuators;
    };

    struct RK4Output {
        dynamics::RigidBodyState Xt1;
        dynamics::WrenchSet WB_set;
    };

    template <typename T, typename Model>
    dynamics::WrenchSet_T<T> compute_wrench_set_T(const Model& model, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& atm, const actuators::ActuatorInputs_T<T>& u, const propulsion::PropellerOmegaDotSet_T<T>& propeller_omega_dot_set, const atmospheric::Wind& windB, const constants::Vector3_T<T>& gB);

    dynamics::Position trans_kin(const dynamics::Position& pt, const dynamics::TranslationalVelocity& vt, const dynamics::TranslationalAcceleration& at, double dt);

    dynamics::OrientationQuaternion quat_kin(const dynamics::OrientationQuaternion& qIB_t, const dynamics::AngularVelocity& wB_BI_t, double dt);

    dynamics::TranslationalVelocity trans_kin_vel(const dynamics::TranslationalVelocity& xt_dot, const dynamics::TranslationalAcceleration& xt_ddot, double dt);

    dynamics::RigidBodyState step_rigid_body(const dynamics::RigidBodyState& Xt, const dynamics::Mass& mass, const dynamics::InertiaTensor& JB, const dynamics::Wrench& WB_net_t, double dt);

    RK4Output step_rigid_body_rk4(const dynamics::RigidBodyState& Xt, RK4Model& model, const operating::OperatingConditions& conditions, const actuators::ActuatorInputs_T<double>& u, double dt);

}

#include "simulation/integrators/public.tpp"
