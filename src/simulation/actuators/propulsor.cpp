#include "simulation/actuators/propulsor.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/transforms/transforms.hpp"

namespace actuators {

    PropulsorActuator::PropulsorActuator(
        double limit_max, 
        double limit_min, 
        double tau, 
        double inclination_angle, 
        double toe_angle,
        const Eigen::Vector3d& pB_prop_cg
    ) : 
        Actuator(limit_max, limit_min, tau),
        inclination_angle(inclination_angle),
        toe_angle(toe_angle),
        pB_prop_cg(pB_prop_cg)
    {
        Eigen::Matrix3d RBP = transforms::eul_to_R(toe_angle, inclination_angle, 0, "ZYX");
        n_prop = RBP * constants::ei;
    }

}
