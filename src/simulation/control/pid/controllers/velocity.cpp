
#include "simulation/control/pid/controllers/velocity.hpp"

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

    PIDControllerInput VelocityPID::make_pid_controller_input(const VelocityControllerInput& controller_input){
        dynamics::RigidBodyState zN_t = controller_input.zN_t;
        actuators::PropulsorActuators propulsor_actuators = controller_input.propulsor_actuators;
        guidance::VelocitySetpoint setpoint = controller_input.setpoint;

        double limit_max_overall = propulsor_actuators.front_propulsor.limit_max + propulsor_actuators.left_propulsor.limit_max + propulsor_actuators.right_propulsor.limit_max;
        double limit_min_overall = propulsor_actuators.front_propulsor.limit_min + propulsor_actuators.left_propulsor.limit_min + propulsor_actuators.right_propulsor.limit_min;

        return {
            .meas = zN_t.v.data(0),
            .meas_des = setpoint.vB_BI.data(0),
            .limit_max = limit_max_overall,
            .limit_min = limit_min_overall
        };
    }

    ControlOutput VelocityPID::step(const VelocityControllerInput& controller_input) {
        SurfaceActuatorInputs_T<double> u_surface{};
        PropulsorActuatorInputs_T<double> u_propulsor{};

        double T_tot = policy.step(
            make_pid_controller_input(controller_input)
        );

        auto [T_front, T_left, T_right] = allocate_thrust(T_tot, controller_input.propulsor_actuators);

        u_propulsor.front_propulsor_cmd = T_front;
        u_propulsor.left_propulsor_cmd = T_left;
        u_propulsor.right_propulsor_cmd = T_right;

        return { u_surface, u_propulsor };
    }
}
