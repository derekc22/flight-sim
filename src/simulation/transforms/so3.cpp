#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include "simulation/constants/constants.hpp"
#include "simulation/util/util.hpp"
#include "simulation/transforms/so3.hpp"
#include "simulation/transforms/s3.hpp"

namespace transforms {



    Eigen::Matrix3d Rx(double phi) {
        Eigen::Matrix3d Rx;
        Rx <<   1,              0,               0,
                0,  std::cos(phi),  -std::sin(phi),
                0,  std::sin(phi),   std::cos(phi);
        return Rx;
    };

    Eigen::Matrix3d Ry(double theta) {
        Eigen::Matrix3d Ry;
        Ry <<   std::cos(theta),  0,  std::sin(theta),
                              0,  1,                0,
               -std::sin(theta),  0,  std::cos(theta);
        return Ry;
    };

    Eigen::Matrix3d Rz(double psi) {
        Eigen::Matrix3d Rz;
        Rz <<   std::cos(psi),  -std::sin(psi),  0,
                std::sin(psi),   std::cos(psi),  0,
                            0,               0,  1;
        return Rz;
    };

    Eigen::Matrix3d _eul_to_R_extr(double a, double b, double c, const std::string& order){
        if (order == "ZYX") return Rx(c) * Ry(b) * Rz(a);
        if (order == "ZXY") return Ry(c) * Rx(b) * Rz(a);

        if (order == "YZX") return Rx(c) * Rz(b) * Ry(a);
        if (order == "YXZ") return Rz(c) * Rx(b) * Ry(a);

        if (order == "XZY") return Ry(c) * Rz(b) * Rx(a);
        if (order == "XYZ") return Rz(c) * Ry(b) * Rx(a);

        if (order == "ZYZ") return Rz(c) * Ry(b) * Rz(a);
        if (order == "ZXZ") return Rz(c) * Rx(b) * Rz(a);

        if (order == "XZX") return Rx(c) * Rz(b) * Rx(a);
        if (order == "XYX") return Rx(c) * Ry(b) * Rx(a);

        if (order == "YZY") return Ry(c) * Rz(b) * Ry(a);
        if (order == "YXY") return Ry(c) * Rx(b) * Ry(a);

        else throw std::invalid_argument("Unsupported Euler order: " + order);
    };

    // All vector rotations (as opposed to frame rotations/coordinate transformations) are, by definition, extrinsic. The concept of an "intrinsic" rotation only applies to frames
    // That is, the concept of an "intrinsic" vector rotation is not defined
    // So the function '_eul_to_R_intr' does not technically make sense
    // However, for consistency with the extrinsic case, it is still implemented since transposing the result of this function indeed gives the correct result for an intrinsic frame rotation/coordinate transformation
    Eigen::Matrix3d _eul_to_R_intr(double a, double b, double c, const std::string& order){
        return _eul_to_R_extr(-a, -b, -c, order).transpose();
    }

    Eigen::Matrix3d _eul_to_C_extr(double a, double b, double c, const std::string& order){
       return _eul_to_R_extr(a, b, c, order).transpose();
    };

    Eigen::Matrix3d _eul_to_C_intr(double a, double b, double c, const std::string& order){
        return _eul_to_R_intr(a, b, c, order).transpose();
    };

    Eigen::Vector3d R_to_eul_extr(const Eigen::Matrix3d& R, const std::string& order) {
        double a = 0.0, b = 0.0, c = 0.0;

        // Tait-Bryan, middle angle in [-pi/2, pi/2]
        if (order == "ZYX") {
            b = std::asin(util::clamp_to_1(R(0,2)));
            const double cb = std::cos(b);
            if (std::abs(cb) > constants::eps) {
                a = std::atan2(-R(0,1), R(0,0));
                c = std::atan2(-R(1,2), R(2,2));
            } else {
                a = std::atan2(R(1,0), R(1,1));
                c = 0.0;
            }

        } else if (order == "ZXY") {
            b = std::asin(util::clamp_to_1(-R(1,2)));
            const double cb = std::cos(b);
            if (std::abs(cb) > constants::eps) {
                a = std::atan2(R(1,0), R(1,1));
                c = std::atan2(R(0,2), R(2,2));
            } else {
                a = std::atan2(-R(0,1), R(0,0));
                c = 0.0;
            }

        } else if (order == "YZX") {
            b = std::asin(util::clamp_to_1(-R(0,1)));
            const double cb = std::cos(b);
            if (std::abs(cb) > constants::eps) {
                a = std::atan2(R(0,2), R(0,0));
                c = std::atan2(R(2,1), R(1,1));
            } else {
                a = std::atan2(-R(2,0), R(2,2));
                c = 0.0;
            }

        } else if (order == "YXZ") {
            b = std::asin(util::clamp_to_1(R(2,1)));
            const double cb = std::cos(b);
            if (std::abs(cb) > constants::eps) {
                a = std::atan2(-R(2,0), R(2,2));
                c = std::atan2(-R(0,1), R(1,1));
            } else {
                a = std::atan2(R(0,2), R(0,0));
                c = 0.0;
            }

        } else if (order == "XZY") {
            b = std::asin(util::clamp_to_1(R(1,0)));
            const double cb = std::cos(b);
            if (std::abs(cb) > constants::eps) {
                a = std::atan2(-R(1,2), R(1,1));
                c = std::atan2(-R(2,0), R(0,0));
            } else {
                a = std::atan2(R(2,1), R(2,2));
                c = 0.0;
            }

        } else if (order == "XYZ") {
            b = std::asin(util::clamp_to_1(-R(2,0)));
            const double cb = std::cos(b);
            if (std::abs(cb) > constants::eps) {
                a = std::atan2(R(2,1), R(2,2));
                c = std::atan2(R(1,0), R(0,0));
            } else {
                a = std::atan2(-R(1,2), R(1,1));
                c = 0.0;
            }

        // Proper Euler (repeated axis), middle angle in [-pi, 0]
        } else if (order == "ZXZ") {
            b = -std::acos(util::clamp_to_1(R(2,2)));
            const double sb = std::sin(b);
            if (std::abs(sb) > constants::eps) {
                a = std::atan2(-R(2,0), -R(2,1));
                c = std::atan2(-R(0,2),  R(1,2));
            } else {
                a = std::atan2(-R(0,1), R(0,0));
                c = 0.0;
            }

        } else if (order == "ZYZ") {
            b = -std::acos(util::clamp_to_1(R(2,2)));
            const double sb = std::sin(b);
            if (std::abs(sb) > constants::eps) {
                a = std::atan2(-R(2,1),  R(2,0));
                c = std::atan2(-R(1,2), -R(0,2));
            } else {
                a = std::atan2(-R(0,1), R(0,0));
                c = 0.0;
            }

        } else if (order == "XYX") {
            b = -std::acos(util::clamp_to_1(R(0,0)));
            const double sb = std::sin(b);
            if (std::abs(sb) > constants::eps) {
                a = std::atan2(-R(0,1), -R(0,2));
                c = std::atan2(-R(1,0),  R(2,0));
            } else {
                a = std::atan2(-R(1,2), R(1,1));
                c = 0.0;
            }

        } else if (order == "XZX") {
            b = -std::acos(util::clamp_to_1(R(0,0)));
            const double sb = std::sin(b);
            if (std::abs(sb) > constants::eps) {
                a = std::atan2(-R(0,2),  R(0,1));
                c = std::atan2(-R(2,0), -R(1,0));
            } else {
                a = std::atan2(-R(1,2), R(1,1));
                c = 0.0;
            }

        } else if (order == "YXY") {
            b = -std::acos(util::clamp_to_1(R(1,1)));
            const double sb = std::sin(b);
            if (std::abs(sb) > constants::eps) {
                a = std::atan2(-R(1,0),  R(1,2));
                c = std::atan2(-R(0,1), -R(2,1));
            } else {
                a = std::atan2(-R(2,0), R(0,0));
                c = 0.0;
            }

        } else if (order == "YZY") {
            b = -std::acos(util::clamp_to_1(R(1,1)));
            const double sb = std::sin(b);
            if (std::abs(sb) > constants::eps) {
                a = std::atan2(-R(1,2), -R(1,0));
                c = std::atan2(-R(2,1),  R(0,1));
            } else {
                a = std::atan2(-R(2,0), R(0,0));
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

    Eigen::Vector3d _C_to_eul_extr(const Eigen::Matrix3d& C, const std::string& order) {
        // C_extr(a, b, c) = R_extr(a, b, c).T = R_intr(-a, -b, -c)
        return R_to_eul_extr(C.transpose(), order);
    }

    Eigen::Vector3d _C_to_eul_intr(const Eigen::Matrix3d& C, const std::string& order) {
        // C_intr(a, b, c) = R_intr(a, b, c).T = R_extr(-a, -b, -c)
        return R_to_eul_intr(C.transpose(), order);
    }


    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    // Whether the net matrix represents an intrinsic or extrinsic rotation depends on whether the matrices passed to the function represent active rotations (ie vector rotations) or passive rotations (ie frame rotations/coordinate transformations/coordinate transformations)
    // Active: extrinsic -> pre-multiply, intrinsic -> not defined
    // Passive: extrinsic -> post-multiply, intrinsic -> pre-multiply
    // All vector rotations (as opposed to frame rotations/coordinate transformations) are, by definition, extrinsic. The concept of an "intrinsic" rotation only applies to frames
    // That is, the concept of an "intrinsic" vector rotation is not defined
    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    Eigen::Matrix3d chain_rot_post(const std::vector<Eigen::Matrix3d>& rot_list) {
        Eigen::Matrix3d rot_tot = constants::I3;
        for (const auto& rot : rot_list){
            rot_tot *= rot;
        }
        return rot_tot;
    }

    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    Eigen::Matrix3d chain_rot_pre(const std::vector<Eigen::Matrix3d>& rot_list) {
        Eigen::Matrix3d rot_tot = constants::I3;

        for (auto rit = rot_list.rbegin(); rit != rot_list.rend(); ++rit) {
            rot_tot *= *rit;
        }
        return rot_tot;
    }

    Eigen::Matrix3d C_from_R(const Eigen::Matrix3d& R){
        return R.transpose();
    }

    Eigen::Matrix3d R_from_C(const Eigen::Matrix3d& C){
        return C.transpose();
    }


    Eigen::Matrix3d eul_to_C(double a, double b, double c, const std::string& order, const std::string& type){
        if (type == "extr") return _eul_to_C_extr(a, b, c, order);
        if (type == "intr") return _eul_to_C_intr(a, b, c, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }

    // Recall from above that the premise of an "intrinsic" vector rotation (as opposed to an intrinsic frame rotations/coordinate transformations) is rejected
    // Thus, the "intr" branch of `eul_to_R` does not exist
    // That is, this function DOES NOT have a 'type' argument and appropriately ONLY calls `_eul_to_R_extr` internally
    // Once again, the 'types' argument is omitted in eul_to_R because including it would imply that active rotation matrices, R, can apply intrinsic rotations
    // Once again, this is not true. Active rotation matrices CANNOT apply intrinsic rotations - they can ONLY apply extrinsic rotations, hence why `eul_to_R` automtically calls _eul_to_R_extr and does not support the option to build an `intrinsic` R (which, again, DOES NOT EXIST)
    Eigen::Matrix3d eul_to_R(double a, double b, double c, const std::string& order){
        return _eul_to_R_extr(a, b, c, order);
    }



    Eigen::Vector3d C_to_eul(const Eigen::Matrix3d& C, const std::string& order, const std::string& type) {
        if (type == "extr") return _C_to_eul_extr(C, order);
        if (type == "intr") return _C_to_eul_intr(C, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }


    Eigen::Quaterniond rot_to_quat(const Eigen::Matrix3d& rot) {
        return transforms::normalize_and_canonicalize(Eigen::Quaterniond(rot));
    }

}



