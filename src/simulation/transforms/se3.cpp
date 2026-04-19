#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include "simulation/constants/constants.hpp"
#include "simulation/transforms/se3.hpp"

namespace transforms {

    Eigen::Matrix3d C_from_H(const Eigen::Matrix4d& H){
        return H.block<3,3>(0,0);
    }

    Eigen::Matrix3d R_from_H(const Eigen::Matrix4d& H){
        return H.block<3,3>(0,0);
    }

    Eigen::Vector3d d_from_H(const Eigen::Matrix4d& H){
        return H.block<3,1>(0,3);
    }

    Eigen::Vector3d p_from_H(const Eigen::Matrix4d& H){
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


    Eigen::Matrix4d make_HR(const Eigen::Matrix3d& R, const Eigen::Vector3d& d, const std::string& first) {
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

    Eigen::Matrix4d make_HC(const Eigen::Matrix3d& C, const Eigen::Vector3d& d, const std::string& first) {
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

    Eigen::Vector3d apply_H(const Eigen::Matrix4d& H, const Eigen::Vector3d& v) {
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
    // Whether the net homogenous transformation matrix represents an intrinsic or extrinsic rotation depends on whether the homogenous transformation matrices passed to the function represent active rotations (ie vector rotations) or passive rotations (ie frame rotations/coordinate transformations/coordinate transformations)
    // Active: extrinsic -> pre-multiply, intrinsic -> not defined
    // Passive: extrinsic -> post-multiply, intrinsic -> pre-multiply
    // All vector rotations (as opposed to frame rotations/coordinate transformations) are, by definition, extrinsic. The concept of an "intrinsic" rotation only applies to frames
    // That is, the concept of an "intrinsic" vector rotation is not defined
    // Given the orientation (of the frame/vector) obtained via the nth transformation, how is the n+1 transformation applied
    Eigen::Matrix4d chain_hom_post(const std::vector<Eigen::Matrix4d>& H_list) {
        Eigen::Matrix4d Htot = constants::HI;
        for (const auto& H : H_list){
            Htot *= H;
        }
        return Htot;
    }

    // Given the orientation (of the frame/vector) obtained via the nth transformation, how is the n+1 transformation applied
    Eigen::Matrix4d chain_hom_pre(const std::vector<Eigen::Matrix4d>& H_list) {
        Eigen::Matrix4d Htot = constants::HI;

        for (auto rit = H_list.rbegin(); rit != H_list.rend(); ++rit) {
            Htot *= *rit;
        }
        return Htot;
    }

}



