#include <Eigen/Dense>
#include <stdexcept>
#include <algorithm>
#include <string>
#include <vector>
#include "simulation/constants/public.hpp"
#include "simulation/transforms/se3/public.hpp"
#include "simulation/transforms/se3/private.hpp"

namespace transforms {

    Eigen::Matrix3d C_from_H(const Eigen::Matrix4d& H) {
        return H.block<3,3>(0,0);
    }

    Eigen::Matrix3d R_from_H(const Eigen::Matrix4d& H) {
        return H.block<3,3>(0,0);
    }

    Eigen::Vector3d d_from_H(const Eigen::Matrix4d& H) {
        return H.block<3,1>(0,3);
    }

    Eigen::Vector3d p_from_H(const Eigen::Matrix4d& H) {
        Eigen::Vector3d Cd = -H.block<3,1>(0,3);
        Eigen::Matrix3d C = H.block<3,3>(0,0);
        return C.transpose() * Cd;
    }
 
    Eigen::Matrix4d make_HR(const Eigen::Matrix3d& R, const Eigen::Vector3d& d, const std::string& first) {
        // d is initially provided in the fixed frame (ie the only frame) and stays in the fixed frame (again, the only frame)
        if (first == "rotate") return make_HR_rotate_first(R, d);
        if (first == "translate") return make_HR_translate_first(R, d);
        throw std::invalid_argument("Unsupported argument value: " + first);
    };

    Eigen::Matrix4d make_HC(const Eigen::Matrix3d& C, const Eigen::Vector3d& d, const std::string& first) {
        // d is initially provided in frame {0} and stays in/attached to frame {0}
        if (first == "rotate") return make_HC_rotate_first(C, d);
        if (first == "translate") return make_HC_translate_first(C, d);
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

}
