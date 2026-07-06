#pragma once
#include <tuple>
#include "simulation/dynamics/public.hpp"

namespace structural { struct StructuralProperties; } // forward declare
namespace aerodynamics { struct AerodynamicProperties; } // forward declare
namespace actuators { struct PropulsorActuators; template <typename T> struct SurfaceActuatorInputs_T; template <typename T> struct PropulsorActuatorInputs_T; } // forward declare
namespace operating { struct OperatingConditions; } // forward declare

namespace integrators {

    struct RK4Model {
        const structural::StructuralProperties& structural;
        const aerodynamics::AerodynamicProperties& aerodynamic;
        actuators::PropulsorActuators& propulsor_actuators;
    };

    struct RigidBodyStateDot {
        dynamics::TranslationalVelocity p_dot;
        dynamics::TranslationalAcceleration v_dot;
        dynamics::OrientationQuaternionRate q_dot;
        dynamics::AngularAcceleration w_dot;
    };

    dynamics::RigidBodyState step_rigid_body(const dynamics::RigidBodyState& XB_BI_t, const dynamics::Mass& mass, const dynamics::InertiaTensor& JB, const dynamics::Wrench& WB_net_t, double dt);

    std::tuple<dynamics::RigidBodyState, dynamics::Wrench> step_rigid_body_rk4(const dynamics::RigidBodyState& XB_BI_t, RK4Model& model, const operating::OperatingConditions& conditions, const actuators::SurfaceActuatorInputs_T<double>& u_surface, const actuators::PropulsorActuatorInputs_T<double>& u_propulsor, double dt);

}
