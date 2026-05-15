#include <Eigen/Dense>
#include <stdexcept>
#include <algorithm>
#include <string>
#include <vector>
#include "simulation/constants/public.hpp"
#include "simulation/util/public.hpp"
#include "simulation/transforms/so3/public.hpp"
#include "simulation/transforms/so3/private.hpp"

namespace transforms {



    Eigen::Matrix3d Rx(double phi) {
        Eigen::Matrix3d Rx;
        Rx <<   1,               0,                0,
                0,  util::cos(phi),  -util::sin(phi),
                0,  util::sin(phi),   util::cos(phi);
        return Rx;
    };

    Eigen::Matrix3d Ry(double theta) {
        Eigen::Matrix3d Ry;
        Ry <<   util::cos(theta),  0,  util::sin(theta),
                               0,  1,                 0,
               -util::sin(theta),  0,  util::cos(theta);
        return Ry;
    };

    Eigen::Matrix3d Rz(double psi) {
        Eigen::Matrix3d Rz;
        Rz <<   util::cos(psi),  -util::sin(psi),  0,
                util::sin(psi),   util::cos(psi),  0,
                             0,               0,   1;
        return Rz;
    };

    Eigen::Matrix3d eul_to_R_extr(double a, double b, double c, const std::string& order){
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
    // So the function 'eul_to_R_intr' does not technically make sense
    // However, for consistency with the extrinsic case, it is still implemented since transposing the result of this function indeed gives the correct result for an intrinsic frame rotation/coordinate transformation
    Eigen::Matrix3d eul_to_R_intr(double a, double b, double c, const std::string& order){
        return eul_to_R_extr(-a, -b, -c, order).transpose();
    }

    Eigen::Matrix3d eul_to_C_extr(double a, double b, double c, const std::string& order){
       return eul_to_R_extr(a, b, c, order).transpose();
    };

    Eigen::Matrix3d eul_to_C_intr(double a, double b, double c, const std::string& order){
        return eul_to_R_intr(a, b, c, order).transpose();
    };

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

}
