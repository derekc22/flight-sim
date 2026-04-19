#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include "simulation/constants/constants.hpp"
#include "simulation/transforms/so3.hpp"
#include "simulation/transforms/s3.hpp"

namespace transforms {

    /** @deprecated */
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

    Eigen::Quaterniond _eul_to_quatR_extr(double a, double b, double c, const std::string& order) {
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

    // All vector rotations (as opposed to frame rotations/coordinate transformations) are, by definition, extrinsic. The concept of an "intrinsic" rotation only applies to frames
    // That is, the concept of an "intrinsic" vector rotation is not defined
    // So, as with _eul_to_R_intr, the function '_eul_to_quatR_intr' does not technically make sense
    // However, for consistency with the extrinsic case, it is still implemented since transposing the result of this function indeed gives the correct result for an intrinsic frame rotation/coordinate transformation
    Eigen::Quaterniond _eul_to_quatR_intr(double a, double b, double c, const std::string& order) {
        return normalize_and_canonicalize(_eul_to_quatR_extr(-a, -b, -c, order).conjugate());
    }

    Eigen::Quaterniond _eul_to_quatC_extr(double a, double b, double c, const std::string& order) {
        return normalize_and_canonicalize(_eul_to_quatR_extr(a,b,c,order).conjugate());
    }

    Eigen::Quaterniond _eul_to_quatC_intr(double a, double b, double c, const std::string& order) {
        return normalize_and_canonicalize(_eul_to_quatR_intr(a,b,c,order).conjugate());
    }


    /** @deprecated */
    // Eigen::Matrix3d quat_to_rot(const Eigen::Quaterniond& q_in){
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


    Eigen::Vector3d quatR_to_eul_intr(const Eigen::Quaterniond& q, const std::string& order){
        Eigen::Matrix3d R = quat_to_rot(q);
        return R_to_eul_intr(R, order);
    }

    Eigen::Vector3d _quatC_to_eul_intr(const Eigen::Quaterniond& q, const std::string& order){
        Eigen::Matrix3d C = quat_to_rot(q);
        return _C_to_eul_intr(C, order);
    }

    Eigen::Vector3d quatR_to_eul_extr(const Eigen::Quaterniond& q, const std::string& order){
        Eigen::Matrix3d R = quat_to_rot(q);
        return R_to_eul_extr(R, order);
    }

    Eigen::Vector3d _quatC_to_eul_extr(const Eigen::Quaterniond& q, const std::string& order){
        Eigen::Matrix3d C = quat_to_rot(q);
        return _C_to_eul_extr(C, order);
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
    // Whether the net quaternion represents an intrinsic or extrinsic rotation depends on whether the quaternions passed to the function represent active rotations (ie vector rotations) or passive rotations (ie frame rotations/coordinate transformations/coordinate transformations)
    // Active: extrinsic -> pre-multiply, intrinsic -> not defined
    // Passive: extrinsic -> post-multiply, intrinsic -> pre-multiply
    // All vector rotations (as opposed to frame rotations/coordinate transformations) are, by definition, extrinsic. The concept of an "intrinsic" rotation only applies to frames
    // That is, the concept of an "intrinsic" vector rotation is not defined
    Eigen::Quaterniond chain_quat_post(const std::vector<Eigen::Quaterniond>& q_list) {
        Eigen::Quaterniond qtot = constants::qI;
        for (const auto& q : q_list){
            qtot *= normalize_and_canonicalize(q);
        }
        return normalize_and_canonicalize(qtot);
    }

    // Given the orientation (of the frame/vector) obtained via the nth rotation, how is the n+1 rotation applied
    Eigen::Quaterniond chain_quat_pre(const std::vector<Eigen::Quaterniond>& q_list) {
        Eigen::Quaterniond qtot = constants::qI;

        for (auto rit = q_list.rbegin(); rit != q_list.rend(); ++rit) {
            qtot *= normalize_and_canonicalize(*rit);
        }
        return normalize_and_canonicalize(qtot);
    }


    Eigen::Quaterniond eul_to_quatR(double a, double b, double c, const std::string& order, const std::string& type){
        if (type == "extr") return _eul_to_quatR_extr(a, b, c, order);
        if (type == "intr") return _eul_to_quatR_intr(a, b, c, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }

    Eigen::Quaterniond eul_to_quatC(double a, double b, double c, const std::string& order, const std::string& type){
        if (type == "extr") return _eul_to_quatC_extr(a, b, c, order);
        if (type == "intr") return _eul_to_quatC_intr(a, b, c, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }

    Eigen::Vector3d quatC_to_eul(const Eigen::Quaterniond& qC, const std::string& order, const std::string& type) {
        if (type == "extr") return _quatC_to_eul_extr(qC, order);
        if (type == "intr") return _quatC_to_eul_intr(qC, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }

    Eigen::Matrix3d quat_to_rot(const Eigen::Quaterniond& q) {
        return Eigen::Matrix3d(transforms::normalize_and_canonicalize(q));
    }



}



