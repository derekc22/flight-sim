
#include <tuple>
#include "simulation/actuators/propulsor/public.hpp"
#include "simulation/actuators/surface/public.hpp"
#include "simulation/control/pid/controllers/velocity/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/guidance/public.hpp"
#include "simulation/util/public.hpp"

namespace control {

    VelocityPID::VelocityPID(const VelocityPIDParameters& params) :
        params(params), policy(params)
    {};

    std::tuple<double, double, double> VelocityPID::allocate_thrust(double T_tot, const actuators::PropulsorActuators& propulsor_actuators) {
        const actuators::PropulsorActuator& front = propulsor_actuators.front_propulsor;
        const actuators::PropulsorActuator& left = propulsor_actuators.left_propulsor;
        const actuators::PropulsorActuator& right = propulsor_actuators.right_propulsor;

        double T_min = front.limit_min + left.limit_min + right.limit_min;
        double T_max = front.limit_max + left.limit_max + right.limit_max;
        double T_cmd = util::clamp(T_tot, T_min, T_max);

        int n = 0;
        if (front.limit_max > front.limit_min) { n++; }
        if (left.limit_max > left.limit_min) { n++; }
        if (right.limit_max > right.limit_min) { n++; }
        if (n == 0) { return { front.limit_min, left.limit_min, right.limit_min }; }

        double T_each = T_cmd / n;

        double T_front = util::clamp(T_each, front.limit_min, front.limit_max);
        double T_left = util::clamp(T_each, left.limit_min, left.limit_max);
        double T_right = util::clamp(T_each, right.limit_min, right.limit_max);

        double T_remaining = T_cmd - T_front - T_left - T_right;

        T_front += util::clamp(T_remaining, 0.0, front.limit_max - T_front);
        T_remaining = T_cmd - T_front - T_left - T_right;

        T_left += util::clamp(T_remaining, 0.0, left.limit_max - T_left);
        T_remaining = T_cmd - T_front - T_left - T_right;

        T_right += util::clamp(T_remaining, 0.0, right.limit_max - T_right);

        return { T_front, T_left, T_right };
    }

    PIDPolicyInput VelocityPID::make_pid_policy_input(const VelocityControllerInput& input) {
        dynamics::RigidBodyState Zt = input.Zt;
        actuators::PropulsorActuators propulsor_actuators = input.propulsor_actuators;
        guidance::VelocitySetpoint setpoint = input.setpoint;

        double limit_max_overall = propulsor_actuators.front_propulsor.limit_max + propulsor_actuators.left_propulsor.limit_max + propulsor_actuators.right_propulsor.limit_max;
        double limit_min_overall = propulsor_actuators.front_propulsor.limit_min + propulsor_actuators.left_propulsor.limit_min + propulsor_actuators.right_propulsor.limit_min;

        return {
            .x = Zt.v.data(0),
            .x_des = setpoint.vB_BI.data(0),
            .limit_min = limit_min_overall,
            .limit_max = limit_max_overall
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
