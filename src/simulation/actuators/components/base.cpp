#include <Eigen/Core>
#include <optional>
#include "simulation/actuators/public/components/base.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/transforms/public/detail/so3.hpp"
#include "simulation/util/public.hpp"

namespace actuators {

    Actuator::Actuator(double limit_max, double limit_min, double tau) : limit_max(limit_max), limit_min(limit_min), tau(tau) {}

    Actuator::Actuator() : Actuator(0.0, 0.0, 0.0) {};

    double Actuator::step(double cmd, double dt) {
        double cmd_clamped = util::clamp(cmd, limit_min, limit_max);
        double init_lag_state = util::clamp(0.0, limit_min, limit_max);

        double previous_cmd = prev_cmd ? prev_cmd.value() : init_lag_state;
        double cmd_lagged = util::first_order_lag(cmd_clamped, previous_cmd, tau, dt);
        prev_cmd = cmd_lagged;
        return cmd_lagged;
    }

    PropulsorActuator::PropulsorActuator(double limit_max, double limit_min, double tau, double inclination_angle, double toe_angle, const Eigen::Vector3d& pB_propB, std::optional<PropellerAssembly> propellers) :
        Actuator(limit_max, limit_min, tau),
        inclination_angle(inclination_angle),
        toe_angle(toe_angle),
        pB_propB(pB_propB),
        propellers(propellers)
    {
        Eigen::Matrix3d RBP = transforms::eul_to_R(toe_angle, inclination_angle, 0.0, transforms::EulerOrder::ZYX);
        n_prop = RBP * constants::ei;
    }

    PropulsorActuator::PropulsorActuator() : PropulsorActuator(0.0, 0.0, 0.0, 0.0, 0.0, constants::Zero3) {}

}
