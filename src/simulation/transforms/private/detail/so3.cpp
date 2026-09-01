#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include "simulation/constants/public.hpp"
#include "simulation/util/public.hpp"
#include "simulation/util/trig/public.hpp"
#include "simulation/transforms/private/detail/so3.hpp"
#include "simulation/transforms/public/detail/so3.hpp"

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

    Eigen::Matrix3d eul_to_R_extr(double a, double b, double c, EulerOrder order) {
        switch (order) {
            case EulerOrder::ZYX: return Rx(c) * Ry(b) * Rz(a);
            case EulerOrder::ZXY: return Ry(c) * Rx(b) * Rz(a);

            case EulerOrder::YZX: return Rx(c) * Rz(b) * Ry(a);
            case EulerOrder::YXZ: return Rz(c) * Rx(b) * Ry(a);

            case EulerOrder::XZY: return Ry(c) * Rz(b) * Rx(a);
            case EulerOrder::XYZ: return Rz(c) * Ry(b) * Rx(a);

            case EulerOrder::ZYZ: return Rz(c) * Ry(b) * Rz(a);
            case EulerOrder::ZXZ: return Rz(c) * Rx(b) * Rz(a);

            case EulerOrder::XZX: return Rx(c) * Rz(b) * Rx(a);
            case EulerOrder::XYX: return Rx(c) * Ry(b) * Rx(a);

            case EulerOrder::YZY: return Ry(c) * Rz(b) * Ry(a);
            case EulerOrder::YXY: return Ry(c) * Rx(b) * Ry(a);

        }
        throw std::invalid_argument("Unsupported Euler order");
    };

    // All vector rotations (as opposed to frame rotations) are, by definition, extrinsic. The concept of an "intrinsic" rotation only applies to frames
    // That is, the concept of an "intrinsic" vector rotation is not defined
    // So the function 'eul_to_R_intr' does not technically make sense
    // However, for consistency with the extrinsic case, it is still implemented since transposing the result of this function indeed gives the correct result for an intrinsic frame rotation/coordinate transformation
    Eigen::Matrix3d eul_to_R_intr(double a, double b, double c, EulerOrder order) {
        return eul_to_R_extr(-a, -b, -c, order).transpose();
    }

    Eigen::Matrix3d eul_to_C_extr(double a, double b, double c, EulerOrder order) {
       return eul_to_R_extr(a, b, c, order).transpose();
    };

    Eigen::Matrix3d eul_to_C_intr(double a, double b, double c, EulerOrder order) {
        return eul_to_R_intr(a, b, c, order).transpose();
    };

    Eigen::Vector3d R_to_eul_extr(const Eigen::Matrix3d& R, EulerOrder order) {
        double a = 0.0, b = 0.0, c = 0.0;

        // Tait-Bryan, middle angle in [-pi/2, pi/2]
        switch (order) {
            case EulerOrder::ZYX: {
                b = util::asin(util::clamp_to_1(R(0,2)));
                double cb = util::cos(b);
                if (std::abs(cb) > constants::eps) {
                    a = util::atan2(-R(0,1), R(0,0));
                    c = util::atan2(-R(1,2), R(2,2));
                } else {
                    a = util::atan2(R(1,0), R(1,1));
                    c = 0.0;
                }
                break;
            }
            case EulerOrder::ZXY: {
                b = util::asin(util::clamp_to_1(-R(1,2)));
                double cb = util::cos(b);
                if (std::abs(cb) > constants::eps) {
                    a = util::atan2(R(1,0), R(1,1));
                    c = util::atan2(R(0,2), R(2,2));
                } else {
                    a = util::atan2(-R(0,1), R(0,0));
                    c = 0.0;
                }
                break;
            }
            case EulerOrder::YZX: {
                b = util::asin(util::clamp_to_1(-R(0,1)));
                double cb = util::cos(b);
                if (std::abs(cb) > constants::eps) {
                    a = util::atan2(R(0,2), R(0,0));
                    c = util::atan2(R(2,1), R(1,1));
                } else {
                    a = util::atan2(-R(2,0), R(2,2));
                    c = 0.0;
                }
                break;
            }
            case EulerOrder::YXZ: {
                b = util::asin(util::clamp_to_1(R(2,1)));
                double cb = util::cos(b);
                if (std::abs(cb) > constants::eps) {
                    a = util::atan2(-R(2,0), R(2,2));
                    c = util::atan2(-R(0,1), R(1,1));
                } else {
                    a = util::atan2(R(0,2), R(0,0));
                    c = 0.0;
                }
                break;
            }
            case EulerOrder::XZY: {
                b = util::asin(util::clamp_to_1(R(1,0)));
                double cb = util::cos(b);
                if (std::abs(cb) > constants::eps) {
                    a = util::atan2(-R(1,2), R(1,1));
                    c = util::atan2(-R(2,0), R(0,0));
                } else {
                    a = util::atan2(R(2,1), R(2,2));
                    c = 0.0;
                }
                break;
            }
            case EulerOrder::XYZ: {
                b = util::asin(util::clamp_to_1(-R(2,0)));
                double cb = util::cos(b);
                if (std::abs(cb) > constants::eps) {
                    a = util::atan2(R(2,1), R(2,2));
                    c = util::atan2(R(1,0), R(0,0));
                } else {
                    a = util::atan2(-R(1,2), R(1,1));
                    c = 0.0;
                }
                break;
            }
            // Proper Euler (repeated axis), middle angle in [-pi, 0]
            case EulerOrder::ZYZ: {
                b = -util::acos(util::clamp_to_1(R(2,2)));
                double sb = util::sin(b);
                if (std::abs(sb) > constants::eps) {
                    a = util::atan2(-R(2,1),  R(2,0));
                    c = util::atan2(-R(1,2), -R(0,2));
                } else {
                    a = util::atan2(-R(0,1), R(0,0));
                    c = 0.0;
                }
                break;
            }
            case EulerOrder::ZXZ: {
                b = -util::acos(util::clamp_to_1(R(2,2)));
                double sb = util::sin(b);
                if (std::abs(sb) > constants::eps) {
                    a = util::atan2(-R(2,0), -R(2,1));
                    c = util::atan2(-R(0,2),  R(1,2));
                } else {
                    a = util::atan2(-R(0,1), R(0,0));
                    c = 0.0;
                }
                break;
            }
            case EulerOrder::YZY: {
                b = -util::acos(util::clamp_to_1(R(1,1)));
                double sb = util::sin(b);
                if (std::abs(sb) > constants::eps) {
                    a = util::atan2(-R(1,2), -R(1,0));
                    c = util::atan2(-R(2,1),  R(0,1));
                } else {
                    a = util::atan2(-R(2,0), R(0,0));
                    c = 0.0;
                }
                break;
            }
            case EulerOrder::YXY: {
                b = -util::acos(util::clamp_to_1(R(1,1)));
                double sb = util::sin(b);
                if (std::abs(sb) > constants::eps) {
                    a = util::atan2(-R(1,0),  R(1,2));
                    c = util::atan2(-R(0,1), -R(2,1));
                } else {
                    a = util::atan2(-R(2,0), R(0,0));
                    c = 0.0;
                }
                break;
            }
            case EulerOrder::XZX: {
                b = -util::acos(util::clamp_to_1(R(0,0)));
                double sb = util::sin(b);
                if (std::abs(sb) > constants::eps) {
                    a = util::atan2(-R(0,2),  R(0,1));
                    c = util::atan2(-R(2,0), -R(1,0));
                } else {
                    a = util::atan2(-R(1,2), R(1,1));
                    c = 0.0;
                }
                break;
            }
            case EulerOrder::XYX: {
                b = -util::acos(util::clamp_to_1(R(0,0)));
                double sb = util::sin(b);
                if (std::abs(sb) > constants::eps) {
                    a = util::atan2(-R(0,1), -R(0,2));
                    c = util::atan2(-R(1,0),  R(2,0));
                } else {
                    a = util::atan2(-R(1,2), R(1,1));
                    c = 0.0;
                }
                break;
            }
            default:
                throw std::invalid_argument("Unsupported Euler order");
        }

        return Eigen::Vector3d(util::wrap_to_pi(a),util::wrap_to_pi(b),util::wrap_to_pi(c));
    }

    Eigen::Vector3d R_to_eul_intr(const Eigen::Matrix3d& R, EulerOrder order) {
        // R_intr(a, b, c) = R_extr(-a, -b, -c).T
        return -1 * R_to_eul_extr(R.transpose(), order);
    }

    Eigen::Vector3d C_to_eul_extr(const Eigen::Matrix3d& C, EulerOrder order) {
        // C_extr(a, b, c) = R_extr(a, b, c).T = R_intr(-a, -b, -c)
        return R_to_eul_extr(C.transpose(), order);
    }

    Eigen::Vector3d C_to_eul_intr(const Eigen::Matrix3d& C, EulerOrder order) {
        // C_intr(a, b, c) = R_intr(a, b, c).T = R_extr(-a, -b, -c)
        return R_to_eul_intr(C.transpose(), order);
    }

    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    // Whether the net matrix represents an intrinsic or extrinsic rotation depends on whether the matrices passed to the function represent active rotations (ie vector rotations) or passive rotations (ie frame rotations)
    // Active: extrinsic -> pre-multiply; intrinsic -> not defined
    // Passive: extrinsic -> post-multiply; intrinsic -> pre-multiply
    // All vector rotations (as opposed to frame rotations) are, by definition, extrinsic. The concept of an "intrinsic" rotation only applies to frames
    // That is, the concept of an "intrinsic" vector rotation is not defined
    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    Eigen::Matrix3d chain_rot_post(const std::vector<Eigen::Matrix3d>& rot_list) {
        Eigen::Matrix3d rot_tot = constants::I3;
        for (const auto& rot : rot_list) {
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
