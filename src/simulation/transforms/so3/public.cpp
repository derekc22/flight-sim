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

    Eigen::Vector3d R_to_eul_extr(const Eigen::Matrix3d& R, const std::string& order) {
        double a = 0.0, b = 0.0, c = 0.0;

        // Tait-Bryan, middle angle in [-pi/2, pi/2]
        if (order == "ZYX") {
            b = util::asin(util::clamp_to_1(R(0,2)));
            double cb = util::cos(b);
            if (std::abs(cb) > constants::eps) {
                a = util::atan2(-R(0,1), R(0,0));
                c = util::atan2(-R(1,2), R(2,2));
            } else {
                a = util::atan2(R(1,0), R(1,1));
                c = 0.0;
            }

        } else if (order == "ZXY") {
            b = util::asin(util::clamp_to_1(-R(1,2)));
            double cb = util::cos(b);
            if (std::abs(cb) > constants::eps) {
                a = util::atan2(R(1,0), R(1,1));
                c = util::atan2(R(0,2), R(2,2));
            } else {
                a = util::atan2(-R(0,1), R(0,0));
                c = 0.0;
            }

        } else if (order == "YZX") {
            b = util::asin(util::clamp_to_1(-R(0,1)));
            double cb = util::cos(b);
            if (std::abs(cb) > constants::eps) {
                a = util::atan2(R(0,2), R(0,0));
                c = util::atan2(R(2,1), R(1,1));
            } else {
                a = util::atan2(-R(2,0), R(2,2));
                c = 0.0;
            }

        } else if (order == "YXZ") {
            b = util::asin(util::clamp_to_1(R(2,1)));
            double cb = util::cos(b);
            if (std::abs(cb) > constants::eps) {
                a = util::atan2(-R(2,0), R(2,2));
                c = util::atan2(-R(0,1), R(1,1));
            } else {
                a = util::atan2(R(0,2), R(0,0));
                c = 0.0;
            }

        } else if (order == "XZY") {
            b = util::asin(util::clamp_to_1(R(1,0)));
            double cb = util::cos(b);
            if (std::abs(cb) > constants::eps) {
                a = util::atan2(-R(1,2), R(1,1));
                c = util::atan2(-R(2,0), R(0,0));
            } else {
                a = util::atan2(R(2,1), R(2,2));
                c = 0.0;
            }

        } else if (order == "XYZ") {
            b = util::asin(util::clamp_to_1(-R(2,0)));
            double cb = util::cos(b);
            if (std::abs(cb) > constants::eps) {
                a = util::atan2(R(2,1), R(2,2));
                c = util::atan2(R(1,0), R(0,0));
            } else {
                a = util::atan2(-R(1,2), R(1,1));
                c = 0.0;
            }

        // Proper Euler (repeated axis), middle angle in [-pi, 0]
        } else if (order == "ZXZ") {
            b = -util::acos(util::clamp_to_1(R(2,2)));
            double sb = util::sin(b);
            if (std::abs(sb) > constants::eps) {
                a = util::atan2(-R(2,0), -R(2,1));
                c = util::atan2(-R(0,2),  R(1,2));
            } else {
                a = util::atan2(-R(0,1), R(0,0));
                c = 0.0;
            }

        } else if (order == "ZYZ") {
            b = -util::acos(util::clamp_to_1(R(2,2)));
            double sb = util::sin(b);
            if (std::abs(sb) > constants::eps) {
                a = util::atan2(-R(2,1),  R(2,0));
                c = util::atan2(-R(1,2), -R(0,2));
            } else {
                a = util::atan2(-R(0,1), R(0,0));
                c = 0.0;
            }

        } else if (order == "XYX") {
            b = -util::acos(util::clamp_to_1(R(0,0)));
            double sb = util::sin(b);
            if (std::abs(sb) > constants::eps) {
                a = util::atan2(-R(0,1), -R(0,2));
                c = util::atan2(-R(1,0),  R(2,0));
            } else {
                a = util::atan2(-R(1,2), R(1,1));
                c = 0.0;
            }

        } else if (order == "XZX") {
            b = -util::acos(util::clamp_to_1(R(0,0)));
            double sb = util::sin(b);
            if (std::abs(sb) > constants::eps) {
                a = util::atan2(-R(0,2),  R(0,1));
                c = util::atan2(-R(2,0), -R(1,0));
            } else {
                a = util::atan2(-R(1,2), R(1,1));
                c = 0.0;
            }

        } else if (order == "YXY") {
            b = -util::acos(util::clamp_to_1(R(1,1)));
            double sb = util::sin(b);
            if (std::abs(sb) > constants::eps) {
                a = util::atan2(-R(1,0),  R(1,2));
                c = util::atan2(-R(0,1), -R(2,1));
            } else {
                a = util::atan2(-R(2,0), R(0,0));
                c = 0.0;
            }

        } else if (order == "YZY") {
            b = -util::acos(util::clamp_to_1(R(1,1)));
            double sb = util::sin(b);
            if (std::abs(sb) > constants::eps) {
                a = util::atan2(-R(1,2), -R(1,0));
                c = util::atan2(-R(2,1),  R(0,1));
            } else {
                a = util::atan2(-R(2,0), R(0,0));
                c = 0.0;
            }

        } else {
            throw std::invalid_argument("Unsupported Euler order: " + order);
        }

        return Eigen::Vector3d(util::wrap_to_pi(a),util::wrap_to_pi(b),util::wrap_to_pi(c));
    }

    Eigen::Vector3d R_to_eul_intr(const Eigen::Matrix3d& R, const std::string& order) {
        // R_intr(a, b, c) = R_extr(-a, -b, -c).T
        return -1 * R_to_eul_extr(R.transpose(), order);
    }

    Eigen::Vector3d C_to_eul_extr(const Eigen::Matrix3d& C, const std::string& order) {
        // C_extr(a, b, c) = R_extr(a, b, c).T = R_intr(-a, -b, -c)
        return R_to_eul_extr(C.transpose(), order);
    }

    Eigen::Vector3d C_to_eul_intr(const Eigen::Matrix3d& C, const std::string& order) {
        // C_intr(a, b, c) = R_intr(a, b, c).T = R_extr(-a, -b, -c)
        return R_to_eul_intr(C.transpose(), order);
    }


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
