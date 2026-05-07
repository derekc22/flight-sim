#pragma once
#include <optional>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "simulation/actuators/actuator.hpp"
#include "simulation/constants/constants.hpp"

namespace control { struct PropulsorActuatorInputs; } // forward declare

namespace actuators {

    struct PropellerAssembly {
        std::vector<std::string> geometry_ids;
        double spin_sign;
        double thrust_coeff;
        double torque_coeff;
        double diameter;
        double spin_inertia;
        std::optional<double> prev_omega;
    };

    struct PropulsorActuator : Actuator {
        double inclination_angle;
        double toe_angle;
        Eigen::Vector3d pB_prop_cg;
        Eigen::Vector3d n_prop;
        std::optional<PropellerAssembly> propellers;

        PropulsorActuator(double limit_max, double limit_min, double tau, double inclination_angle, double toe_angle, const Eigen::Vector3d& pB_prop_cg, std::optional<PropellerAssembly> propellers = std::nullopt);
    };

    struct FrontPropulsor : PropulsorActuator { using PropulsorActuator::PropulsorActuator; };
    struct LeftPropulsor : PropulsorActuator { using PropulsorActuator::PropulsorActuator; };
    struct RightPropulsor : PropulsorActuator { using PropulsorActuator::PropulsorActuator; };

    struct PropulsorActuators {
        FrontPropulsor front_propulsor;
        LeftPropulsor left_propulsor;
        RightPropulsor right_propulsor;
    };

}
