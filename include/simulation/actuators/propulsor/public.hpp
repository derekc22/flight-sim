#pragma once
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <optional>
#include "simulation/actuators/actuator/public.hpp"
#include "simulation/constants/public.hpp"

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
        Eigen::Vector3d p_propulsor_cg;
        Eigen::Vector3d n_prop;
        std::optional<PropellerAssembly> propellers;

        PropulsorActuator(double limit_max, double limit_min, double tau, double inclination_angle, double toe_angle, const Eigen::Vector3d& p_propulsor_cg, std::optional<PropellerAssembly> propellers = std::nullopt);
        PropulsorActuator();
    };

    struct FrontPropulsor : PropulsorActuator { using PropulsorActuator::PropulsorActuator; };
    struct LeftPropulsor : PropulsorActuator { using PropulsorActuator::PropulsorActuator; };
    struct RightPropulsor : PropulsorActuator { using PropulsorActuator::PropulsorActuator; };

    struct PropulsorActuators {
        FrontPropulsor front_propulsor;
        LeftPropulsor left_propulsor;
        RightPropulsor right_propulsor;
    };

    template <typename T>
    struct PropulsorActuatorInputs_T {
        T front_propulsor_cmd = T(0.0);
        T left_propulsor_cmd = T(0.0);
        T right_propulsor_cmd = T(0.0);
    };

    template <typename T>
    using PropulsorActuatorInputsVector_T = Eigen::Matrix<T, constants::full_propulsor_input_dim, 1>;

    PropulsorActuatorInputsVector_T<double> unpack_propulsor_actuator_inputs(const PropulsorActuatorInputs_T<double>& u_propulsor);

}
