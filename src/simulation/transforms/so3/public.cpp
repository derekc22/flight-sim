#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <string>
#include <vector>
#include "simulation/constants/public.hpp"
#include "simulation/util/public.hpp"
#include "simulation/transforms/s3/public.hpp"
#include "simulation/transforms/so3/public.hpp"
#include "simulation/transforms/so3/private.hpp"

namespace transforms {

    Eigen::Matrix3d C_from_R(const Eigen::Matrix3d& R){
        return R.transpose();
    }

    Eigen::Matrix3d R_from_C(const Eigen::Matrix3d& C){
        return C.transpose();
    }


    Eigen::Matrix3d eul_to_C(double a, double b, double c, const std::string& order, const std::string& type){
        if (type == "extr") return eul_to_C_extr(a, b, c, order);
        if (type == "intr") return eul_to_C_intr(a, b, c, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }

    // Recall from above that the premise of an "intrinsic" vector rotation (as opposed to an intrinsic frame rotations/coordinate transformations) is rejected
    // Thus, the "intr" branch of `eul_to_R` does not exist
    // That is, this function DOES NOT have a 'type' argument and appropriately ONLY calls `eul_to_R_extr` internally
    // Once again, the 'types' argument is omitted in eul_to_R because including it would imply that active rotation matrices, R, can apply intrinsic rotations
    // Once again, this is not true. Active rotation matrices CANNOT apply intrinsic rotations - they can ONLY apply extrinsic rotations, hence why `eul_to_R` automtically calls eul_to_R_extr and does not support the option to build an `intrinsic` R (which, again, DOES NOT EXIST)
    Eigen::Matrix3d eul_to_R(double a, double b, double c, const std::string& order){
        return eul_to_R_extr(a, b, c, order);
    }

    Eigen::Vector3d C_to_eul(const Eigen::Matrix3d& C, const std::string& order, const std::string& type) {
        if (type == "extr") return C_to_eul_extr(C, order);
        if (type == "intr") return C_to_eul_intr(C, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }


    Eigen::Quaterniond rot_to_quat(const Eigen::Matrix3d& rot) {
        return transforms::normalize_and_canonicalize(Eigen::Quaterniond(rot));
    }

}
