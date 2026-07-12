#pragma once
#include <tuple>
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
        const structural::StructuralProperties& structural;
        const aerodynamics::AerodynamicProperties& aerodynamic;
        actuators::PropulsorActuators& propulsor_actuators;
    };

    struct RigidBodyStateDot {
        dynamics::TranslationalVelocity p_dot;
        dynamics::TranslationalAcceleration v_dot;
        dynamics::AngularAcceleration w_dot;
    };

    struct RK4Output {
        dynamics::RigidBodyState Xt1;
        dynamics::Wrench WB_net;
        dynamics::Wrench WB_aerodynamic;
        dynamics::Wrench WB_propulsive;
    };

    template <typename T>
    struct WrenchSet_T {
        dynamics::Wrench_T<T> aerodynamic;
        dynamics::Wrench_T<T> propulsive;
        dynamics::Wrench_T<T> net;
    };

    template <typename T>
    WrenchSet_T<T> compute_wrench_set_T(const aerodynamics::AerodynamicProperties& aerodynamic_properties, const actuators::PropulsorActuators& propulsor_actuators, const dynamics::Twist_T<T>& twist, const atmospheric::StaticAtmosphericState& atm, const actuators::ActuatorInputs_T<T>& u, const propulsion::PropulsorOmegaDot_T<T>& omega_dot, const atmospheric::Wind& windB, double mass, const constants::Vector3_T<T>& gB);

    dynamics::Position trans_kin(const dynamics::Position& xt, const dynamics::TranslationalVelocity& xt_dot, const dynamics::TranslationalAcceleration& xt_ddot, double dt);

    dynamics::OrientationQuaternion quat_kin(const dynamics::OrientationQuaternion& qIB_t, const dynamics::AngularVelocity& wB_BI_t, double dt);

    dynamics::TranslationalVelocity trans_kin_vel(const dynamics::TranslationalVelocity& xt_dot, const dynamics::TranslationalAcceleration& xt_ddot, double dt);

    dynamics::RigidBodyState step_rigid_body(const dynamics::RigidBodyState& Xt, const dynamics::Mass& mass, const dynamics::InertiaTensor& JB, const dynamics::Wrench& WB_net_t, double dt);

    RK4Output step_rigid_body_rk4(const dynamics::RigidBodyState& Xt, RK4Model& model, const operating::OperatingConditions& conditions, const actuators::SurfaceActuatorInputs_T<double>& u_surface, const actuators::PropulsorActuatorInputs_T<double>& u_propulsor, double dt);

}

#include "simulation/integrators/public.tpp"
