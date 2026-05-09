#include "simulation/constants/constants.hpp"
#include "simulation/actuators/propulsor/propulsor.hpp"
#include "simulation/transforms/transforms.hpp"

namespace actuators {

    PropulsorActuator::PropulsorActuator(double limit_max, double limit_min, double tau, double inclination_angle, double toe_angle, const Eigen::Vector3d& pB_prop_cg, std::optional<PropellerAssembly> propellers) :
        Actuator(limit_max, limit_min, tau),
        inclination_angle(inclination_angle),
        toe_angle(toe_angle),
        pB_prop_cg(pB_prop_cg),
        propellers(propellers)
    {
        Eigen::Matrix3d RBP = transforms::eul_to_R(toe_angle, inclination_angle, 0, "ZYX");
        n_prop = RBP * constants::ei;
    }

    FullPropulsorActuatorInputsVector_T<double> unpack_full_propulsor_actuator_inputs(const PropulsorActuatorInputs_T<double>& u_propulsor) {
        FullPropulsorActuatorInputsVector_T<double> out;
        out << u_propulsor.front_propulsor_cmd,
               u_propulsor.left_propulsor_cmd,
               u_propulsor.right_propulsor_cmd;
        return out;
    }

}
