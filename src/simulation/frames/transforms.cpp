#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <iostream>
#include <vector>
#include "simulation/frames/transforms.hpp"


namespace transforms {

    double deg2rad(double deg){
        return deg * (M_PI / 180.0);
    };

    double rad2deg(double rad){
        return rad * (180.0 / M_PI);
    };

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

    Eigen::Matrix3d eul2rotm_extr(double a, double b, double c, const std::string& order){
        if      (order == "ZYX") return Rx(c) * Ry(b) * Rz(a);
        else if (order == "ZXY") return Ry(c) * Rx(b) * Rz(a);
        else if (order == "YZX") return Rx(c) * Rz(b) * Ry(a);
        else if (order == "YXZ") return Rz(c) * Rx(b) * Ry(a);
        else if (order == "XZY") return Ry(c) * Rz(b) * Rx(a);
        else if (order == "XYZ") return Rz(c) * Ry(b) * Rx(a);

        // Proper Euler (repeated axis)
        else if (order == "ZXZ") return Rz(c) * Rx(b) * Rz(a);
        else if (order == "ZYZ") return Rz(c) * Ry(b) * Rz(a);
        else if (order == "XYX") return Rx(c) * Ry(b) * Rx(a);
        else if (order == "XZX") return Rx(c) * Rz(b) * Rx(a);
        else if (order == "YXY") return Ry(c) * Rx(b) * Ry(a);
        else if (order == "YZY") return Ry(c) * Rz(b) * Ry(a);

        else throw std::invalid_argument("Unsupported Euler order: " + order);
    };

    Eigen::Matrix3d eul2rotm_intr(double a, double b, double c, const std::string& order){
        if      (order == "ZYX") return Rz(a) * Ry(b) * Rx(c);
        else if (order == "ZXY") return Rz(a) * Rx(b) * Ry(c);
        else if (order == "YZX") return Ry(a) * Rz(b) * Rx(c);
        else if (order == "YXZ") return Ry(a) * Rx(b) * Rz(c);
        else if (order == "XZY") return Rx(a) * Rz(b) * Ry(c);
        else if (order == "XYZ") return Rx(a) * Ry(b) * Rz(c);

        // Proper Euler (repeated axis)
        else if (order == "ZXZ") return Rz(a) * Rx(b) * Rz(c);
        else if (order == "ZYZ") return Rz(a) * Ry(b) * Rz(c);
        else if (order == "XYX") return Rx(a) * Ry(b) * Rx(c);
        else if (order == "XZX") return Rx(a) * Rz(b) * Rx(c);
        else if (order == "YXY") return Ry(a) * Rx(b) * Ry(c);
        else if (order == "YZY") return Ry(a) * Rz(b) * Ry(c);

        else throw std::invalid_argument("Unsupported Euler order: " + order);
    }

    double clampTo1(double x) {
        // clamps to [-1, 1]
        return std::clamp(x, -1.0, 1.0);
    }

    double wrapToPi(double x) {
        // maps to (-pi, pi]
        return std::remainder(x, 2.0 * M_PI);
    }

    Eigen::Vector3d rotm2eul_intr(const Eigen::Matrix3d& R, const std::string& order) {
        double eps = 1e-12;

        double a = 0.0, b = 0.0, c = 0.0;

        if (order == "ZYX") {
            b = std::asin(clampTo1(-R(2,0)));
            double cb = std::cos(b);
            if (std::abs(cb) > eps) {
                a = std::atan2(R(1,0), R(0,0));
                c = std::atan2(R(2,1), R(2,2));
            } else {
                a = std::atan2(-R(0,1), R(1,1));
                c = 0.0;
            }
        } else if (order == "ZXY") {
            b = std::asin(clampTo1(R(2,1)));
            double cb = std::cos(b);
            if (std::abs(cb) > eps) {
                a = std::atan2(-R(0,1), R(1,1));
                c = std::atan2(-R(2,0), R(2,2));
            } else {
                a = std::atan2(R(1,0), R(0,0));
                c = 0.0;
            }
        } else if (order == "YZX") {
            b = std::asin(clampTo1(R(1,0)));
            double cb = std::cos(b);
            if (std::abs(cb) > eps) {
                a = std::atan2(-R(2,0), R(0,0));
                c = std::atan2(-R(1,2), R(1,1));
            } else {
                a = std::atan2(R(0,2), R(2,2));
                c = 0.0;
            }
        } else if (order == "YXZ") {
            b = std::asin(clampTo1(-R(1,2)));
            double cb = std::cos(b);
            if (std::abs(cb) > eps) {
                a = std::atan2(R(0,2), R(2,2));
                c = std::atan2(R(1,0), R(1,1));
            } else {
                a = std::atan2(-R(2,0), R(0,0));
                c = 0.0;
            }
        } else if (order == "XZY") {
            b = std::asin(clampTo1(-R(0,1)));
            double cb = std::cos(b);
            if (std::abs(cb) > eps) {
                a = std::atan2(R(2,1), R(1,1));
                c = std::atan2(R(0,2), R(0,0));
            } else {
                a = std::atan2(R(2,0), R(1,0));
                c = 0.0;
            }
        } else if (order == "XYZ") {
            b = std::asin(clampTo1(R(0,2)));
            double cb = std::cos(b);
            if (std::abs(cb) > eps) {
                a = std::atan2(-R(1,2), R(2,2));
                c = std::atan2(-R(0,1), R(0,0));
            } else {
                a = std::atan2(R(2,1), R(1,1));
                c = 0.0;
            }

        // Proper Euler (repeated axis)
        } else if (order == "ZXZ") {
            b = std::acos(clampTo1(R(2,2)));
            double sb = std::sin(b);
            if (std::abs(sb) > eps) {
                a = std::atan2(R(0,2), -R(1,2));
                c = std::atan2(R(2,0),  R(2,1));
            } else {
                a = std::atan2(R(1,0), R(0,0));
                c = 0.0;
            }
        } else if (order == "ZYZ") {
            b = std::acos(clampTo1(R(2,2)));
            double sb = std::sin(b);
            if (std::abs(sb) > eps) {
                a = std::atan2(R(1,2),  R(0,2));
                c = std::atan2(R(2,1), -R(2,0));
            } else {
                a = std::atan2(R(1,0), R(0,0));
                c = 0.0;
            }
        } else if (order == "XYX") {
            b = std::acos(clampTo1(R(0,0)));
            double sb = std::sin(b);
            if (std::abs(sb) > eps) {
                a = std::atan2(R(1,0), -R(2,0));
                c = std::atan2(R(0,1),  R(0,2));
            } else {
                a = std::atan2(R(2,1), R(1,1));
                c = 0.0;
            }
        } else if (order == "XZX") {
            b = std::acos(clampTo1(R(0,0)));
            double sb = std::sin(b);
            if (std::abs(sb) > eps) {
                a = std::atan2(R(2,0),  R(1,0));
                c = std::atan2(R(0,2), -R(0,1));
            } else {
                a = std::atan2(R(2,1), R(1,1));
                c = 0.0;
            }
        } else if (order == "YXY") {
            b = std::acos(clampTo1(R(1,1)));
            double sb = std::sin(b);
            if (std::abs(sb) > eps) {
                a = std::atan2(R(0,1),  R(2,1));
                c = std::atan2(R(1,0), -R(1,2));
            } else {
                a = std::atan2(R(0,2), R(0,0));
                c = 0.0;
            }
        } else if (order == "YZY") {
            b = std::acos(clampTo1(R(1,1)));
            double sb = std::sin(b);
            if (std::abs(sb) > eps) {
                a = std::atan2(R(2,1), -R(0,1));
                c = std::atan2(R(1,2),  R(1,0));
            } else {
                a = std::atan2(R(0,2), R(0,0));
                c = 0.0;
            }

        } else {
            throw std::invalid_argument("Unsupported Euler order: " + order);
        }

        return Eigen::Vector3d(wrapToPi(a), wrapToPi(b), wrapToPi(c));

    }

    Eigen::Vector3d rotm2eul_extr(const Eigen::Matrix3d& R, const std::string& order) {
        if (order.size() != 3) {
            throw std::invalid_argument("Unsupported Euler order: " + order);
        }
        const std::string rev(order.rbegin(), order.rend());
        const Eigen::Vector3d abc_rev = rotm2eul_intr(R, rev);          // (a_rev, b_rev, c_rev)
        return Eigen::Vector3d(abc_rev[2], abc_rev[1], abc_rev[0]);    // (a,b,c)
    }

    // Call this when you want to rotate the vector by R
    Eigen::Vector3d active_rot(const Eigen::Matrix3d& R, const Eigen::Vector3d& v){
        return R * v;
    }

    // Call this when you want to rotate the frame by R
    Eigen::Vector3d passive_rot(const Eigen::Matrix3d& R, const Eigen::Vector3d& v){
        return R.transpose() * v;
    };

    // d is passed by the user as a translation in the local frame (before rotation)
    // along with the frame, d is then rotated
    // the local frame is then translated along this rotated d
    Eigen::Matrix4d _make_H_rotate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d) {
        Eigen::Matrix4d H = Eigen::Matrix4d::Zero();
        H.block<3,3>(0,0) = R;
        H.block<3,1>(0,3) = R*d;
        H(3, 3) = 1.0;
        return H;
    };

    // d is passed by the user as a translation in the local frame (before rotation)
    // only the frame is rotated. d stays un-rotated
    // the local frame is then translated along the original, un-rotated d
    Eigen::Matrix4d _make_H_translate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d) {
        Eigen::Matrix4d H = Eigen::Matrix4d::Zero();
        H.block<3,3>(0,0) = R;
        H.block<3,1>(0,3) = d;
        H(3, 3) = 1.0;
        return H;
    };

    // There is no 'makeH_intr' vs 'makeH_extr'
    // Whether an H encodes an intrinsic vs extrinsic transformation depends entirely on whether the R used to construct it encoded an intrinsic vs extrinsic rotation
    // That it, it is entirely determined by the rotation matrix - Thus there are no conflicts here
    Eigen::Matrix4d makeH(const Eigen::Matrix3d& R, const Eigen::Vector3d& d, const std::string& first) {
        if (first == "rotate") return _make_H_rotate_first(R, d);
        else if (first == "translate") return _make_H_translate_first(R, d);
        else throw std::invalid_argument("Unsupported argument value: " + first);
    };

    Eigen::Matrix4d make_Hinv(const Eigen::Matrix4d& H) {
        Eigen::Matrix4d H_inv = Eigen::Matrix4d::Zero();
        Eigen::Matrix3d R = H.block<3,3>(0,0);
        Eigen::Vector3d d = H.block<3,1>(0,3);
        
        H_inv.block<3,3>(0,0) = R.transpose();
        H_inv.block<3,1>(0,3) = -R.transpose() * d;
        H_inv(3, 3) = 1.0;
        return H_inv;
    };

    // Call this when you want to transform the vector
    Eigen::Vector3d active_hom(const Eigen::Matrix4d& H, const Eigen::Vector3d& v) {
        Eigen::Vector4d P = Eigen::Vector4d::Zero();
        P.head(3) = v;
        P(3) = 1;
        return (H * P).head(3);
    };

    // Call this when you want to transform the frame
    Eigen::Vector3d passive_hom(const Eigen::Matrix4d& H, const Eigen::Vector3d& v) {
        Eigen::Vector4d P = Eigen::Vector4d::Zero();
        P.head(3) = v;
        P(3) = 1;
        return (make_Hinv(H) * P).head(3);
    };

    Eigen::Matrix4d _identity_hom() {
        return Eigen::Matrix4d::Identity();
    }

    // Given the orientation (of the frame/vector) obtained via the nth transformation, how is the n+1 transformation applied
    Eigen::Matrix4d chain_hom_intr(const std::vector<Eigen::Matrix4d>& H_list) {
        Eigen::Matrix4d Htot = _identity_hom();
        for (const auto& H : H_list){
            Htot *= H;
        }
        return Htot;
    }

    // Given the orientation (of the frame/vector) obtained via the nth transformation, how is the n+1 transformation applied
    Eigen::Matrix4d chain_hom_extr(const std::vector<Eigen::Matrix4d>& H_list) {
        Eigen::Matrix4d Htot = _identity_hom();

        for (auto rit = H_list.rbegin(); rit != H_list.rend(); ++rit) {
            Htot *= *rit;
        }
        return Htot;
    }

    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    Eigen::Matrix3d chain_rot_intr(const std::vector<Eigen::Matrix3d>& R_list) {
        Eigen::Matrix3d Rtot = Eigen::Matrix3d::Identity();
        for (const auto& R : R_list){
            Rtot *= R;
        }
        return Rtot;
    }

    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    Eigen::Matrix3d chain_rot_extr(const std::vector<Eigen::Matrix3d>& R_list) {
        Eigen::Matrix3d Rtot = Eigen::Matrix3d::Identity();

        for (auto rit = R_list.rbegin(); rit != R_list.rend(); ++rit) {
            Rtot *= *rit;
        }
        return Rtot;
    }

    // Deprecated
    Eigen::Quaterniond quat_mul(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
        double w1 = q1.w(), x1 = q1.x(), y1 = q1.y(), z1 = q1.z();
        double w2 = q2.w(), x2 = q2.x(), y2 = q2.y(), z2 = q2.z();

        Eigen::Quaterniond q;
        q.w() = w1*w2 - x1*x2 - y1*y2 - z1*z2;
        q.x() = w1*x2 + x1*w2 + y1*z2 - z1*y2;
        q.y() = w1*y2 - x1*z2 + y1*w2 + z1*x2;
        q.z() = w1*z2 + x1*y2 - y1*x2 + z1*w2;
        return q;
    }



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

    Eigen::Quaterniond _normalize_and_canonicalize(Eigen::Quaterniond q) {
        q.normalize();
        // Canonicalize sign: q and -q represent the same rotation
        if (q.w() < 0.0) q.coeffs() *= -1.0; // coeffs() is (x,y,z,w)
        return q;
    }

    Eigen::Quaterniond eul2quat_extr(double a, double b, double c, const std::string& order) {
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

        return _normalize_and_canonicalize(q);
    }

    Eigen::Quaterniond eul2quat_intr(double a, double b, double c, const std::string& order) {
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

        return _normalize_and_canonicalize(q);
    }




    Eigen::Matrix3d quat2rot(const Eigen::Quaterniond& q_in){
        Eigen::Quaterniond q = _normalize_and_canonicalize(q_in);

        double w = q.w();
        double x = q.x();
        double y = q.y();
        double z = q.z();

        Eigen::Matrix3d R;

        double xx = x*x, yy = y*y, zz = z*z;
        double xy = x*y, xz = x*z, yz = y*z;
        double wx = w*x, wy = w*y, wz = w*z;

        R(0,0) = 1.0 - 2.0*(yy + zz);
        R(0,1) = 2.0*(xy - wz);
        R(0,2) = 2.0*(xz + wy);

        R(1,0) = 2.0*(xy + wz);
        R(1,1) = 1.0 - 2.0*(xx + zz);
        R(1,2) = 2.0*(yz - wx);

        R(2,0) = 2.0*(xz - wy);
        R(2,1) = 2.0*(yz + wx);
        R(2,2) = 1.0 - 2.0*(xx + yy);

        return R;
    }


    Eigen::Vector3d quat2eul_intr(const Eigen::Quaterniond& q, const std::string& order){
        Eigen::Matrix3d R = quat2rot(q);
        return rotm2eul_intr(R, order);
    }

    Eigen::Vector3d quat2eul_extr(const Eigen::Quaterniond& q, const std::string& order){
        Eigen::Matrix3d R = quat2rot(q);
        return rotm2eul_extr(R, order);
    }

    // Call this when you want to rotate the vector
    Eigen::Vector3d active_quat(const Eigen::Quaterniond& q, const Eigen::Vector3d& v) {
        return _normalize_and_canonicalize(q) * v;
    };

    // Call this when you want to rotate the frame
    Eigen::Vector3d passive_quat(const Eigen::Quaterniond& q, const Eigen::Vector3d& v) {
        Eigen::Quaterniond q_conj = _normalize_and_canonicalize(q).conjugate();
        return q_conj * v;
    };


    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    Eigen::Quaterniond chain_quat_intr(const std::vector<Eigen::Quaterniond>& q_list) {
        Eigen::Quaterniond qtot = Eigen::Quaterniond::Identity();
        for (const auto& q : q_list){
            _normalize_and_canonicalize(q);
            qtot *= q;
        }
        return qtot;
    }

    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    Eigen::Quaterniond chain_quat_extr(const std::vector<Eigen::Quaterniond>& q_list) {
        Eigen::Quaterniond qtot = Eigen::Quaterniond::Identity();

        for (auto rit = q_list.rbegin(); rit != q_list.rend(); ++rit) {
            qtot *= _normalize_and_canonicalize(*rit);
        }
        return qtot;
    }



}



