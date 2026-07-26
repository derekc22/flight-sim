
#include <tuple>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/control/pid/controllers/velocity/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/util/public.hpp"

namespace control {

    VelocityPID::VelocityPID(const VelocityPIDParameters& params) :
        policy(params)
    {};

    PIDPolicyInput VelocityPID::make_pid_policy_input(const VelocityControllerInput& input) {
        dynamics::RigidBodyState Zt = input.Zt;
        actuators::PropulsorActuators propulsor_actuators = input.propulsor_actuators;
        guidance::VelocitySetpoint setpoint = input.setpoint;

        return {
            .x = Zt.v.data(0),
            .x_des = setpoint.vB_BI.data(0)
        };
    }

    ControlOutput VelocityPID::step(const VelocityControllerInput& input, double dt) {
        actuators::SurfaceActuatorInputs_T<double> u_surface{};
        actuators::PropulsorActuatorInputs_T<double> u_propulsor{};

        double T_tot = policy.step(
            make_pid_policy_input(input),
            dt
        );

        auto [T_front, T_left, T_right] = allocate_thrust(T_tot, input.propulsor_actuators);

        u_propulsor.front_propulsor_cmd = T_front;
        u_propulsor.left_propulsor_cmd = T_left;
        u_propulsor.right_propulsor_cmd = T_right;

        return { u_surface, u_propulsor };
    }
}
