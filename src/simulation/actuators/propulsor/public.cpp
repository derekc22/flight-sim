#include <Eigen/Core>
#include <optional>
#include <string>
#include "simulation/transforms/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/actuators/propulsor/public.hpp"

namespace actuators {

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

    PropulsorActuatorInputsVector unpack_propulsor_actuator_inputs(const PropulsorActuatorInputs_T<double>& u_propulsor) {
        PropulsorActuatorInputsVector out;
        out << u_propulsor.front_propulsor_cmd,
               u_propulsor.left_propulsor_cmd,
               u_propulsor.right_propulsor_cmd;
        return out;
    }
}
