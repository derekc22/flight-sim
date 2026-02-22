#include <Eigen/Dense>
#include <vector>

namespace structures {

    struct Component {
        double mass;
        double x_size;
        double y_size;
        double z_size;
        double x_loc;
        double y_loc;
        double z_loc;
    }

    struct Structure {
        std::vector<Structure> components;
        double Mass;
        Eigen::Vector3d cg_loc;
        Eigen::Matrix3d J;

        Structure(c) : components(c);

        double compute_Mass();
        Eigen::Vector3d compute_CG();
        Eigen::Matrix3d compute_J();

    };


}