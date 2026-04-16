#pragma once
#include <Eigen/Dense>


namespace transforms {

    /**
    This description @deprecated because H can either be a passive or active transformation based on your interpretation of what the input C/R do
    * @brief Creates a homogenous transformation matrix
    *
    * @details Given a rotation matrix “R” and a translation vector "d" whose combined action transforms frame {0} into frame {1},
               this function returns a homogenous transformation matrix "H" representing that transformation
    *
    * @param R Rotation matrix that rotates frame {0} into frame {1}
    * @param d Translation vector "d" that translates frame {0} into frame {1}
    * @param first Specifies which transformation occurs first. Options:
                    "rotate": The rotation matrix "R" is applied first. Translation occurs about the rotated vector R*d
                    "translate": The translation vector "d" is applied first. Translation occurs about the original vector d
    *
    * @return Homogenous transformation matrix "H" that transforms frame {0} into frame {1}
    */
    Eigen::Matrix4d make_HR(const Eigen::Matrix3d& R, const Eigen::Vector3d& d, const std::string& first);

    /**
    * @brief Creates an inverse homogenous transformation matrix
    *
    * @details Given a homogenous transformation matrix “H" that transforms frame {0} into frame {1},
            this function returns the inverse homogenous transformation matrix "H^-1" that transforms frame {1} back into frame {0}
    *
    * @param H Homogenous transformation matrix that transforms frame {0} into frame {1}
    *
    * @return Homogenous transformation matrix "H^-1" that transforms frame {1} back into frame {0}
    */
    Eigen::Matrix4d make_Hinv(const Eigen::Matrix4d& H);

    /**
    * @brief Applies a homogenous transformation matrix
    *
    * @details Given a homogenous transformation matrix “H" that transforms frame {0} into frame {1} and a vector "p" expressed WRT frame {1}
            this function returns the inverse homogenous transformation matrix "H^-1" that transforms frame {1} back into frame {0}
    *
    * @param H Homogenous transformation matrix that transforms frame {0} into frame {1}
    *
    * @return Homogenous transformation matrix "H^-1" that transforms frame {1} back into frame {0}
    */

    // /**
    // * @brief Performs an active homogenous transformation
    // *
    // * @details Given vector “v0” expressed WRT frame {0} and a homogenous transformation matrix "H" that transforms vector "v0" into vector "v1" WRT frame {0},
    //         this function returns vector "v1" expressed WRT frame {0}
    // *
    // * @param H Homogenous transformation matrix that transforms vector v0 into v1
    // * @param v0 Vector "v0" expressed WRT frame {0}
    // *
    // * @return Vector "v1" expressed WRT frame {0}
    // */
    // Eigen::Vector3d active_hom(const Eigen::Matrix4d& H, const Eigen::Vector3d& v);

    // /**
    // * @brief Performs a passive homogenous transformation
    // *
    // * @details Given vector “p” expressed WRT frame {0} and a homogenous transformation matrix "H" that transforms frame {0} into frame {1},
    //         this function returns vector "p" expressed WRT frame {1}
    // *
    // * @param H Homogenous transformation matrix that transforms frame {0} into frame {1}
    // * @param v Vector "v" expressed WRT frame {0}
    // *
    // * @return Vector "v" expressed WRT frame {1}
    // */
    // Eigen::Vector3d passive_hom(const Eigen::Matrix4d& H, const Eigen::Vector3d& v);

    Eigen::Matrix3d C_from_H(const Eigen::Matrix4d& H);
    Eigen::Matrix3d R_from_H(const Eigen::Matrix4d& H);
    Eigen::Vector3d d_from_H(const Eigen::Matrix4d& H);
    Eigen::Vector3d p_from_H(const Eigen::Matrix4d& H);

    Eigen::Vector3d apply_H(const Eigen::Matrix4d& H, const Eigen::Vector3d& v);
    Eigen::Matrix4d chain_hom_post(const std::vector<Eigen::Matrix4d>& H_list);
    Eigen::Matrix4d chain_hom_pre(const std::vector<Eigen::Matrix4d>& H_list);


    // Internal
    Eigen::Matrix4d _make_HR_rotate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d);
    Eigen::Matrix4d _make_HR_translate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d);

    Eigen::Matrix4d make_HC(const Eigen::Matrix3d& C, const Eigen::Vector3d& d, const std::string& first);
    Eigen::Matrix4d _make_HC_translate_first(const Eigen::Matrix3d& C, const Eigen::Vector3d& d);
    Eigen::Matrix4d _make_HC_rotate_first(const Eigen::Matrix3d& C, const Eigen::Vector3d& d);
}


