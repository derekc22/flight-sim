#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <vector>
#include "simulation/transforms/transforms.hpp"
#include "simulation/global/global.hpp"

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

    Eigen::Matrix3d eul2R_extr(double a, double b, double c, const std::string& order){
        if (order == "ZYX") return Rx(c) * Ry(b) * Rz(a);
        if (order == "ZXY") return Ry(c) * Rx(b) * Rz(a);
        if (order == "YZX") return Rx(c) * Rz(b) * Ry(a);
        if (order == "YXZ") return Rz(c) * Rx(b) * Ry(a);
        if (order == "XZY") return Ry(c) * Rz(b) * Rx(a);
        if (order == "XYZ") return Rz(c) * Ry(b) * Rx(a);

        // Proper Euler (repeated axis)
        if (order == "ZXZ") return Rz(c) * Rx(b) * Rz(a);
        if (order == "ZYZ") return Rz(c) * Ry(b) * Rz(a);
        if (order == "XYX") return Rx(c) * Ry(b) * Rx(a);
        if (order == "XZX") return Rx(c) * Rz(b) * Rx(a);
        if (order == "YXY") return Ry(c) * Rx(b) * Ry(a);
        if (order == "YZY") return Ry(c) * Rz(b) * Ry(a);

        else throw std::invalid_argument("Unsupported Euler order: " + order);
    };

    Eigen::Matrix3d eul2C_extr(double a, double b, double c, const std::string& order){
       return eul2R_extr(a, b, c, order).transpose();
    };

    // All vector rotations (as opposed to frame rotations) are, by definition, extrinsic. The concept of an "intrinsic" rotation only applies to frames
    // That is, the concept of an "intrinsic" vector rotation is not defined
    // So the function 'eul2R_intr' does not technically make sense
    // However, for consistency with the extrinsic case, it is still implemented since transposing the result of this function indeed gives the correct result for an intrinsic frame rotation
    Eigen::Matrix3d eul2R_intr(double a, double b, double c, const std::string& order){
        if (order == "ZYX") return Rz(a) * Ry(b) * Rx(c);
        if (order == "ZXY") return Rz(a) * Rx(b) * Ry(c);
        if (order == "YZX") return Ry(a) * Rz(b) * Rx(c);
        if (order == "YXZ") return Ry(a) * Rx(b) * Rz(c);
        if (order == "XZY") return Rx(a) * Rz(b) * Ry(c);
        if (order == "XYZ") return Rx(a) * Ry(b) * Rz(c);

        // Proper Euler (repeated axis)
        if (order == "ZXZ") return Rz(a) * Rx(b) * Rz(c);
        if (order == "ZYZ") return Rz(a) * Ry(b) * Rz(c);
        if (order == "XYX") return Rx(a) * Ry(b) * Rx(c);
        if (order == "XZX") return Rx(a) * Rz(b) * Rx(c);
        if (order == "YXY") return Ry(a) * Rx(b) * Ry(c);
        if (order == "YZY") return Ry(a) * Rz(b) * Ry(c);

        else throw std::invalid_argument("Unsupported Euler order: " + order);
    }

    Eigen::Matrix3d eul2C_intr(double a, double b, double c, const std::string& order){
        return eul2R_intr(a, b, c, order).transpose();
    };

    Eigen::Vector3d R2eul_intr(const Eigen::Matrix3d& R, const std::string& order) {

        double a = 0.0, b = 0.0, c = 0.0;

        if (order == "ZYX") {
            b = std::asin(global::clampTo1(-R(2,0)));
            double cb = std::cos(b);
            if (std::abs(cb) > global::eps) {
                a = std::atan2(R(1,0), R(0,0));
                c = std::atan2(R(2,1), R(2,2));
            } else {
                a = std::atan2(-R(0,1), R(1,1));
                c = 0.0;
            }
        } else if (order == "ZXY") {
            b = std::asin(global::clampTo1(R(2,1)));
            double cb = std::cos(b);
            if (std::abs(cb) > global::eps) {
                a = std::atan2(-R(0,1), R(1,1));
                c = std::atan2(-R(2,0), R(2,2));
            } else {
                a = std::atan2(R(1,0), R(0,0));
                c = 0.0;
            }
        } else if (order == "YZX") {
            b = std::asin(global::clampTo1(R(1,0)));
            double cb = std::cos(b);
            if (std::abs(cb) > global::eps) {
                a = std::atan2(-R(2,0), R(0,0));
                c = std::atan2(-R(1,2), R(1,1));
            } else {
                a = std::atan2(R(0,2), R(2,2));
                c = 0.0;
            }
        } else if (order == "YXZ") {
            b = std::asin(global::clampTo1(-R(1,2)));
            double cb = std::cos(b);
            if (std::abs(cb) > global::eps) {
                a = std::atan2(R(0,2), R(2,2));
                c = std::atan2(R(1,0), R(1,1));
            } else {
                a = std::atan2(-R(2,0), R(0,0));
                c = 0.0;
            }
        } else if (order == "XZY") {
            b = std::asin(global::clampTo1(-R(0,1)));
            double cb = std::cos(b);
            if (std::abs(cb) > global::eps) {
                a = std::atan2(R(2,1), R(1,1));
                c = std::atan2(R(0,2), R(0,0));
            } else {
                a = std::atan2(R(2,0), R(1,0));
                c = 0.0;
            }
        } else if (order == "XYZ") {
            b = std::asin(global::clampTo1(R(0,2)));
            double cb = std::cos(b);
            if (std::abs(cb) > global::eps) {
                a = std::atan2(-R(1,2), R(2,2));
                c = std::atan2(-R(0,1), R(0,0));
            } else {
                a = std::atan2(R(2,1), R(1,1));
                c = 0.0;
            }

        // Proper Euler (repeated axis)
        } else if (order == "ZXZ") {
            b = std::acos(global::clampTo1(R(2,2)));
            double sb = std::sin(b);
            if (std::abs(sb) > global::eps) {
                a = std::atan2(R(0,2), -R(1,2));
                c = std::atan2(R(2,0),  R(2,1));
            } else {
                a = std::atan2(R(1,0), R(0,0));
                c = 0.0;
            }
        } else if (order == "ZYZ") {
            b = std::acos(global::clampTo1(R(2,2)));
            double sb = std::sin(b);
            if (std::abs(sb) > global::eps) {
                a = std::atan2(R(1,2),  R(0,2));
                c = std::atan2(R(2,1), -R(2,0));
            } else {
                a = std::atan2(R(1,0), R(0,0));
                c = 0.0;
            }
        } else if (order == "XYX") {
            b = std::acos(global::clampTo1(R(0,0)));
            double sb = std::sin(b);
            if (std::abs(sb) > global::eps) {
                a = std::atan2(R(1,0), -R(2,0));
                c = std::atan2(R(0,1),  R(0,2));
            } else {
                a = std::atan2(R(2,1), R(1,1));
                c = 0.0;
            }
        } else if (order == "XZX") {
            b = std::acos(global::clampTo1(R(0,0)));
            double sb = std::sin(b);
            if (std::abs(sb) > global::eps) {
                a = std::atan2(R(2,0),  R(1,0));
                c = std::atan2(R(0,2), -R(0,1));
            } else {
                a = std::atan2(R(2,1), R(1,1));
                c = 0.0;
            }
        } else if (order == "YXY") {
            b = std::acos(global::clampTo1(R(1,1)));
            double sb = std::sin(b);
            if (std::abs(sb) > global::eps) {
                a = std::atan2(R(0,1),  R(2,1));
                c = std::atan2(R(1,0), -R(1,2));
            } else {
                a = std::atan2(R(0,2), R(0,0));
                c = 0.0;
            }
        } else if (order == "YZY") {
            b = std::acos(global::clampTo1(R(1,1)));
            double sb = std::sin(b);
            if (std::abs(sb) > global::eps) {
                a = std::atan2(R(2,1), -R(0,1));
                c = std::atan2(R(1,2),  R(1,0));
            } else {
                a = std::atan2(R(0,2), R(0,0));
                c = 0.0;
            }

        } else {
            throw std::invalid_argument("Unsupported Euler order: " + order);
        }

        return Eigen::Vector3d(global::wrapToPi(a), global::wrapToPi(b), global::wrapToPi(c));

    }

    Eigen::Vector3d R2eul_extr(const Eigen::Matrix3d& R, const std::string& order) {
        if (order.size() != 3) {
            throw std::invalid_argument("Unsupported Euler order: " + order);
        }
        std::string rev(order.rbegin(), order.rend());
        Eigen::Vector3d abc_rev = R2eul_intr(R, rev);          
        return Eigen::Vector3d{abc_rev(2), abc_rev(1), abc_rev(0)};
    }

    Eigen::Vector3d C2eul_extr(const Eigen::Matrix3d& C, const std::string& order) {
        // C_extr(a, b, c) = R_extr(a, b, c).T = R_intr(-a, -b, -c)
        return R2eul_extr(C.transpose(), order);
    }

    Eigen::Vector3d C2eul_intr(const Eigen::Matrix3d& C, const std::string& order) {
        // C_intr(a, b, c) = R_intr(a, b, c).T = R_extr(-a, -b, -c)
        return R2eul_intr(C.transpose(), order);
    }


    Eigen::Matrix3d CfromH(const Eigen::Matrix4d& H){
        return H.block<3,3>(0,0);
    }

    Eigen::Matrix3d RfromH(const Eigen::Matrix4d& H){
        return H.block<3,3>(0,0);
    }

    Eigen::Vector3d dfromH(const Eigen::Matrix4d& H){
        return H.block<3,1>(0,3);
    }

    Eigen::Vector3d pfromH(const Eigen::Matrix4d& H){
        Eigen::Vector3d Cd = -H.block<3,1>(0,3);
        Eigen::Matrix3d C = H.block<3,3>(0,0);
        return C.transpose() * Cd;
    }
 
    Eigen::Matrix4d _make_HR_translate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d) {
        Eigen::Matrix4d H = Eigen::Matrix4d::Zero();
        H.block<3,3>(0,0) = R;
        H.block<3,1>(0,3) = R*d;
        H(3, 3) = 1.0;
        return H;
    };

  
    Eigen::Matrix4d _make_HR_rotate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d) {
        Eigen::Matrix4d H = Eigen::Matrix4d::Zero();
        H.block<3,3>(0,0) = R;
        H.block<3,1>(0,3) = d;
        H(3, 3) = 1.0;
        return H;
    };


    Eigen::Matrix4d makeHR(const Eigen::Matrix3d& R, const Eigen::Vector3d& d, const std::string& first) {
        // d is initially provided in the fixed frame (ie the only frame) and stays in the fixed frame (again, the only frame)
        if (first == "rotate") return _make_HR_rotate_first(R, d);
        if (first == "translate") return _make_HR_translate_first(R, d);
        throw std::invalid_argument("Unsupported argument value: " + first);
    };

    Eigen::Matrix4d _make_HC_rotate_first(const Eigen::Matrix3d& C, const Eigen::Vector3d& d) {
        Eigen::Matrix4d H = Eigen::Matrix4d::Zero();
        H.block<3,3>(0,0) = C;
        H.block<3,1>(0,3) = -d;
        H(3,3) = 1.0;
        return H;
    }

    Eigen::Matrix4d _make_HC_translate_first(const Eigen::Matrix3d& C, const Eigen::Vector3d& d) {
        Eigen::Matrix4d H = Eigen::Matrix4d::Zero();
        H.block<3,3>(0,0) = C;
        H.block<3,1>(0,3) = -C*d;
        H(3,3) = 1.0;
        return H;
    }

    Eigen::Matrix4d makeHC(const Eigen::Matrix3d& C, const Eigen::Vector3d& d, const std::string& first) {
        // d is initially provided in frame {0} and stays in/attached to frame {0}
        if (first == "rotate") return _make_HC_rotate_first(C, d);
        if (first == "translate") return _make_HC_translate_first(C, d);
        throw std::invalid_argument("Unsupported argument value: " + first);
    }


    Eigen::Matrix4d make_Hinv(const Eigen::Matrix4d& H) {
        Eigen::Matrix4d H_inv = Eigen::Matrix4d::Zero();
        Eigen::Matrix3d R = H.block<3,3>(0,0);
        Eigen::Vector3d d = H.block<3,1>(0,3);
        
        H_inv.block<3,3>(0,0) = R.transpose();
        H_inv.block<3,1>(0,3) = -R.transpose() * d;
        H_inv(3, 3) = 1.0;
        return H_inv;
    };

    Eigen::Vector3d apply_hom(const Eigen::Matrix4d& H, const Eigen::Vector3d& v) {
        Eigen::Vector4d P = Eigen::Vector4d::Zero();
        P.head(3) = v;
        P(3) = 1;
        return (H * P).head(3);
    };

    // // Call this when you want to transform the vector
    // Eigen::Vector3d active_hom(const Eigen::Matrix4d& H, const Eigen::Vector3d& v) {
    //     Eigen::Vector4d P = Eigen::Vector4d::Zero();
    //     P.head(3) = v;
    //     P(3) = 1;
    //     return (H * P).head(3);
    // };

    // // Call this when you want to transform the frame
    // Eigen::Vector3d passive_hom(const Eigen::Matrix4d& H, const Eigen::Vector3d& v) {
    //     Eigen::Vector4d P = Eigen::Vector4d::Zero();
    //     P.head(3) = v;
    //     P(3) = 1;
    //     return (make_Hinv(H) * P).head(3);
    // };
    

    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    // Whether the net homogenous transformation matrix represents an intrinsic or extrinsic rotation depends on whether the homogenous transformation matrices passed to the function represent active rotations (ie vector rotations) or passive rotations (ie frame rotations/coordinate transformations)
    // Active: extrinsic -> pre-multiply, intrinsic -> not defined
    // Passive: extrinsic -> post-multiply, intrinsic -> pre-multiply
    // All vector rotations (as opposed to frame rotations) are, by definition, extrinsic. The concept of an "intrinsic" rotation only applies to frames
    // That is, the concept of an "intrinsic" vector rotation is not defined
    // Given the orientation (of the frame/vector) obtained via the nth transformation, how is the n+1 transformation applied
    Eigen::Matrix4d chain_hom_post(const std::vector<Eigen::Matrix4d>& H_list) {
        Eigen::Matrix4d Htot = global::HI;
        for (const auto& H : H_list){
            Htot *= H;
        }
        return Htot;
    }

    // Given the orientation (of the frame/vector) obtained via the nth transformation, how is the n+1 transformation applied
    Eigen::Matrix4d chain_hom_pre(const std::vector<Eigen::Matrix4d>& H_list) {
        Eigen::Matrix4d Htot = global::HI;

        for (auto rit = H_list.rbegin(); rit != H_list.rend(); ++rit) {
            Htot *= *rit;
        }
        return Htot;
    }


    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    // Whether the net matrix represents an intrinsic or extrinsic rotation depends on whether the matrices passed to the function represent active rotations (ie vector rotations) or passive rotations (ie frame rotations/coordinate transformations)
    // Active: extrinsic -> pre-multiply, intrinsic -> not defined
    // Passive: extrinsic -> post-multiply, intrinsic -> pre-multiply
    // All vector rotations (as opposed to frame rotations) are, by definition, extrinsic. The concept of an "intrinsic" rotation only applies to frames
    // That is, the concept of an "intrinsic" vector rotation is not defined
    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    Eigen::Matrix3d chain_rot_post(const std::vector<Eigen::Matrix3d>& rot_list) {
        Eigen::Matrix3d rot_tot = global::I3;
        for (const auto& rot : rot_list){
            rot_tot *= rot;
        }
        return rot_tot;
    }

    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    Eigen::Matrix3d chain_rot_pre(const std::vector<Eigen::Matrix3d>& rot_list) {
        Eigen::Matrix3d rot_tot = global::I3;

        for (auto rit = rot_list.rbegin(); rit != rot_list.rend(); ++rit) {
            rot_tot *= *rit;
        }
        return rot_tot;
    }

    Eigen::Matrix3d CfromR(const Eigen::Matrix3d& R){
        return R.transpose();
    }

    Eigen::Matrix3d RfromC(const Eigen::Matrix3d& C){
        return C.transpose();
    }

    // // Deprecated
    // Eigen::Quaterniond quat_mul(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
    //     double w1 = q1.w(), x1 = q1.x(), y1 = q1.y(), z1 = q1.z();
    //     double w2 = q2.w(), x2 = q2.x(), y2 = q2.y(), z2 = q2.z();

    //     Eigen::Quaterniond q;
    //     q.w() = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    //     q.x() = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    //     q.y() = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    //     q.z() = w1*z2 + x1*y2 - y1*x2 + z1*w2;
    //     return q;
    // }



    Eigen::Quaterniond qx(double phi) {
        double h = 0.5 * phi;
        return Eigen::Quaterniond(std::cos(h), std::sin(h), 0.0, 0.0); // (w,x,y,z)
    }

    Eigen::Quaterniond qy(double theta) {
        double h = 0.5 * theta;
        return Eigen::Quaterniond(std::cos(h), 0.0, std::sin(h), 0.0);
    }

    Eigen::Quaterniond qz(double psi) {
        double h = 0.5 * psi;
        return Eigen::Quaterniond(std::cos(h), 0.0, 0.0, std::sin(h));
    }

    Eigen::Quaterniond normalize_and_canonicalize(Eigen::Quaterniond q) {
        q.normalize();
        // Canonicalize sign: q and -q represent the same rotation
        if (q.w() < 0.0) q.coeffs() *= -1.0; // coeffs() is (x,y,z,w)
        return q;
    }

    Eigen::Quaterniond eul2quatR_extr(double a, double b, double c, const std::string& order) {
        Eigen::Quaterniond q;

        if      (order == "ZYX") q = qx(c) * qy(b) * qz(a);
        else if (order == "ZXY") q = qy(c) * qx(b) * qz(a);
        else if (order == "YZX") q = qx(c) * qz(b) * qy(a);
        else if (order == "YXZ") q = qz(c) * qx(b) * qy(a);
        else if (order == "XZY") q = qy(c) * qz(b) * qx(a);
        else if (order == "XYZ") q = qz(c) * qy(b) * qx(a);

        // Proper Euler (repeated axis)
        else if (order == "ZXZ") q = qz(c) * qx(b) * qz(a);
        else if (order == "ZYZ") q = qz(c) * qy(b) * qz(a);
        else if (order == "XYX") q = qx(c) * qy(b) * qx(a);
        else if (order == "XZX") q = qx(c) * qz(b) * qx(a);
        else if (order == "YXY") q = qy(c) * qx(b) * qy(a);
        else if (order == "YZY") q = qy(c) * qz(b) * qy(a);

        else throw std::invalid_argument("Unsupported Euler order: " + order);

        return normalize_and_canonicalize(q);
    }

    Eigen::Quaterniond eul2quatC_extr(double a, double b, double c, const std::string& order) {
        return normalize_and_canonicalize(eul2quatR_extr(a,b,c,order).conjugate());
    }

    // All vector rotations (as opposed to frame rotations) are, by definition, extrinsic. The concept of an "intrinsic" rotation only applies to frames
    // That is, the concept of an "intrinsic" vector rotation is not defined
    // So, as with eul2R_intr, the function 'eul2quatR_intr' does not technically make sense
    // However, for consistency with the extrinsic case, it is still implemented since transposing the result of this function indeed gives the correct result for an intrinsic frame rotation
    Eigen::Quaterniond eul2quatR_intr(double a, double b, double c, const std::string& order) {
        Eigen::Quaterniond q;

        if      (order == "ZYX") q = qz(a) * qy(b) * qx(c);
        else if (order == "ZXY") q = qz(a) * qx(b) * qy(c);
        else if (order == "YZX") q = qy(a) * qz(b) * qx(c);
        else if (order == "YXZ") q = qy(a) * qx(b) * qz(c);
        else if (order == "XZY") q = qx(a) * qz(b) * qy(c);
        else if (order == "XYZ") q = qx(a) * qy(b) * qz(c);

        // Proper Euler (repeated axis)
        else if (order == "ZXZ") q = qz(a) * qx(b) * qz(c);
        else if (order == "ZYZ") q = qz(a) * qy(b) * qz(c);
        else if (order == "XYX") q = qx(a) * qy(b) * qx(c);
        else if (order == "XZX") q = qx(a) * qz(b) * qx(c);
        else if (order == "YXY") q = qy(a) * qx(b) * qy(c);
        else if (order == "YZY") q = qy(a) * qz(b) * qy(c);

        else throw std::invalid_argument("Unsupported Euler order: " + order);

        return normalize_and_canonicalize(q);
    }

    Eigen::Quaterniond eul2quatC_intr(double a, double b, double c, const std::string& order) {
        return normalize_and_canonicalize(eul2quatR_intr(a,b,c,order).conjugate());
    }


    /** @deprecated */
    // Eigen::Matrix3d quat2rot(const Eigen::Quaterniond& q_in){
    //     Eigen::Quaterniond q = normalize_and_canonicalize(q_in);

    //     double w = q.w();
    //     double x = q.x();
    //     double y = q.y();
    //     double z = q.z();

    //     Eigen::Matrix3d R;

    //     double xx = x*x, yy = y*y, zz = z*z;
    //     double xy = x*y, xz = x*z, yz = y*z;
    //     double wx = w*x, wy = w*y, wz = w*z;

    //     R(0,0) = 1.0 - 2.0*(yy + zz);
    //     R(0,1) = 2.0*(xy - wz);
    //     R(0,2) = 2.0*(xz + wy);

    //     R(1,0) = 2.0*(xy + wz);
    //     R(1,1) = 1.0 - 2.0*(xx + zz);
    //     R(1,2) = 2.0*(yz - wx);

    //     R(2,0) = 2.0*(xz - wy);
    //     R(2,1) = 2.0*(yz + wx);
    //     R(2,2) = 1.0 - 2.0*(xx + yy);

    //     return R;
    // }


    Eigen::Vector3d quatR2eul_intr(const Eigen::Quaterniond& q, const std::string& order){
        Eigen::Matrix3d R = quat2rot(q);
        return R2eul_intr(R, order);
    }

    Eigen::Vector3d quatC2eul_intr(const Eigen::Quaterniond& q, const std::string& order){
        Eigen::Matrix3d C = quat2rot(q);
        return C2eul_intr(C, order);
    }

    Eigen::Vector3d quatR2eul_extr(const Eigen::Quaterniond& q, const std::string& order){
        Eigen::Matrix3d R = quat2rot(q);
        return R2eul_extr(R, order);
    }

    Eigen::Vector3d quatC2eul_extr(const Eigen::Quaterniond& q, const std::string& order){
        Eigen::Matrix3d C = quat2rot(q);
        return C2eul_extr(C, order);
    }

    // // Call this when you want to rotate the vector
    // Eigen::Vector3d active_quat(const Eigen::Quaterniond& q, const Eigen::Vector3d& v) {
    //     return normalize_and_canonicalize(q) * v;
    // };

    // // Call this when you want to rotate the frame
    // Eigen::Vector3d passive_quat(const Eigen::Quaterniond& q, const Eigen::Vector3d& v) {
    //     Eigen::Quaterniond q_conj = normalize_and_canonicalize(q).conjugate();
    //     return q_conj * v;
    // };


    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    // Whether the net quaternion represents an intrinsic or extrinsic rotation depends on whether the quaternions passed to the function represent active rotations (ie vector rotations) or passive rotations (ie frame rotations/coordinate transformations)
    // Active: extrinsic -> pre-multiply, intrinsic -> not defined
    // Passive: extrinsic -> post-multiply, intrinsic -> pre-multiply
    // All vector rotations (as opposed to frame rotations) are, by definition, extrinsic. The concept of an "intrinsic" rotation only applies to frames
    // That is, the concept of an "intrinsic" vector rotation is not defined
    Eigen::Quaterniond chain_quat_post(const std::vector<Eigen::Quaterniond>& q_list) {
        Eigen::Quaterniond qtot = Eigen::Quaterniond::Identity();
        for (const auto& q : q_list){
            qtot *= normalize_and_canonicalize(q);
        }
        return normalize_and_canonicalize(qtot);
    }

    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    Eigen::Quaterniond chain_quat_pre(const std::vector<Eigen::Quaterniond>& q_list) {
        Eigen::Quaterniond qtot = Eigen::Quaterniond::Identity();

        for (auto rit = q_list.rbegin(); rit != q_list.rend(); ++rit) {
            qtot *= normalize_and_canonicalize(*rit);
        }
        return normalize_and_canonicalize(qtot);
    }




    Eigen::Matrix3d eul2C(double a, double b, double c, const std::string& order, const std::string& type){
        if (type == "extr") return eul2C_extr(a, b, c, order);
        if (type == "intr") return eul2C_intr(a, b, c, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }

    Eigen::Matrix3d eul2R(double a, double b, double c, const std::string& order, const std::string& type){
        if (type == "extr") return eul2R_extr(a, b, c, order);
        if (type == "intr") return eul2R_intr(a, b, c, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }

    Eigen::Quaterniond eul2quatR(double a, double b, double c, const std::string& order, const std::string& type){
        if (type == "extr") return eul2quatR_extr(a, b, c, order);
        if (type == "intr") return eul2quatR_intr(a, b, c, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }

    Eigen::Quaterniond eul2quatC(double a, double b, double c, const std::string& order, const std::string& type){
        if (type == "extr") return eul2quatC_extr(a, b, c, order);
        if (type == "intr") return eul2quatC_intr(a, b, c, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }

    Eigen::Vector3d quatC2eul(const Eigen::Quaterniond& qC, const std::string& order, const std::string& type) {
        if (type == "extr") return quatC2eul_extr(qC, order);
        if (type == "intr") return quatC2eul_intr(qC, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }

    Eigen::Vector3d C2eul(const Eigen::Matrix3d& C, const std::string& order, const std::string& type) {
        if (type == "extr") return C2eul_extr(C, order);
        if (type == "intr") return C2eul_intr(C, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }


    Eigen::Quaterniond rot2quat(const Eigen::Matrix3d& rot) {
        return transforms::normalize_and_canonicalize(Eigen::Quaterniond(rot));
    }

    Eigen::Matrix3d quat2rot(const Eigen::Quaterniond& q) {
        return Eigen::Matrix3d(transforms::normalize_and_canonicalize(q));
    }



}



