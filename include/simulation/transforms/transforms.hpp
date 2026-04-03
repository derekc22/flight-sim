#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <tuple>
#include <stdexcept>

namespace transforms {

    /**
    * @brief Returns an active rotation matrix about the "x" axis
    *
    * @details Given an angle "phi" in radians,
            this function returns a rotation matrix "Rx" that encodes a rotation about the "x" axis of a coordinate frame by "phi" radians
    *
    * @param phi Angle to rotate by [radians]
    *
    * @return Rotation matrix "Rx" that encodes a rotation about the 'x' axis of a coordinate frame by "phi" radians
    */
    Eigen::Matrix3d Rx(double phi);

    /**
    * @brief Returns an active rotation matrix about the "y" axis
    *
    * @details Given an angle "theta" in radians,
            this function returns a rotation matrix "Ry" that encodes a rotation about the "y" axis of a coordinate frame by "theta" radians
    *
    * @param theta Angle to rotate by [radians]
    *
    * @return Rotation matrix "Ry" that encodes a rotation about the "y" axis of a coordinate frame by "theta" radians
    */
    Eigen::Matrix3d Ry(double theta);

    /**
    * @brief Returns a active rotation matrix about the "z" axis
    *
    * @details Given an angle "psi" in radians,
            this function returns a rotation matrix "Rz" that encodes a rotation about the "z" axis of a coordinate frame by "psi" radians
    *
    * @param psi Angle to rotate by [radians]
    *
    * @return Rotation matrix "Rz" that encodes a rotation about the "z" axis of a coordinate frame by "psi" radians
    */
    Eigen::Matrix3d Rz(double psi);

    Eigen::Matrix3d eul_to_R_extr(double roll, double pitch, double yaw, const std::string& order);
    Eigen::Matrix3d eul_to_R_intr(double roll, double pitch, double yaw, const std::string& order);

    Eigen::Vector3d R_to_eul_extr(const Eigen::Matrix3d& R, const std::string& order);
    Eigen::Vector3d R_to_eul_intr(const Eigen::Matrix3d& R, const std::string& order);

    // /**
    // * @brief Performs an active rotation
    // *
    // * @details Given vector “v0” expressed WRT frame {0} and a rotation matrix "R" that rotates vector "v0" into vector "v1" WRT frame {0},
    //         this function returns vector "v1" expressed WRT frame {0}
    // *
    // * @param R Rotation matrix that rotates vector v0 into v1
    // * @param v0 Vector "v0" expressed WRT frame {0}
    // *
    // * @return Vector "v1" expressed WRT frame {0}
    // */
    // Eigen::Vector3d active_R(const Eigen::Matrix3d& R, const Eigen::Vector3d& v0);

    // /**
    // * @brief Performs a passive rotation
    // *
    // * @details Given vector “v” expressed WRT frame {0} and a rotation matrix "R" that rotates frame {0} into frame {1},
    //         this function returns vector "v" expressed WRT frame {1}
    // *
    // * @param R Rotation matrix that rotates frame {0} into frame {1}
    // * @param v Vector "v" expressed WRT frame {0}
    // *
    // * @return Vector "v" expressed WRT frame {1}
    // */
    // Eigen::Vector3d passive_R(const Eigen::Matrix3d& R, const Eigen::Vector3d& v);

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

    Eigen::Matrix3d C_from_R(const Eigen::Matrix3d& R);
    Eigen::Matrix3d R_from_C(const Eigen::Matrix3d& C);

    Eigen::Matrix3d C_from_H(const Eigen::Matrix4d& H);
    Eigen::Matrix3d R_from_H(const Eigen::Matrix4d& H);
    Eigen::Vector3d d_from_H(const Eigen::Matrix4d& H);
    Eigen::Vector3d p_from_H(const Eigen::Matrix4d& H);

    Eigen::Vector3d apply_H(const Eigen::Matrix4d& H, const Eigen::Vector3d& v);
    Eigen::Matrix4d chain_hom_post(const std::vector<Eigen::Matrix4d>& H_list);
    Eigen::Matrix4d chain_hom_pre(const std::vector<Eigen::Matrix4d>& H_list);

    Eigen::Matrix3d chain_rot_post(const std::vector<Eigen::Matrix3d>& rot_list);
    Eigen::Matrix3d chain_rot_pre(const std::vector<Eigen::Matrix3d>& rot_list);

    Eigen::Quaterniond eul_to_quatR_extr(double a, double b, double c, const std::string& order);
    Eigen::Quaterniond eul_to_quatR_intr(double a, double b, double c, const std::string& order);
    Eigen::Quaterniond eul_to_quatC_extr(double a, double b, double c, const std::string& order);
    Eigen::Quaterniond eul_to_quatC_intr(double a, double b, double c, const std::string& order);

    Eigen::Quaterniond qx(double phi);
    Eigen::Quaterniond qy(double theta);
    Eigen::Quaterniond qz(double psi);

    // Eigen::Quaterniond quat_mul(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);


    Eigen::Matrix3d quat_to_rot(const Eigen::Quaterniond& q);

    Eigen::Vector3d quatR_to_eul_extr(const Eigen::Quaterniond& q, const std::string& order);
    Eigen::Vector3d quatR_to_eul_intr(const Eigen::Quaterniond& q, const std::string& order);
    Eigen::Vector3d quatC_to_eul_extr(const Eigen::Quaterniond& q, const std::string& order);
    Eigen::Vector3d quatC_to_eul_intr(const Eigen::Quaterniond& q, const std::string& order);

    // Eigen::Vector3d active_quat(const Eigen::Quaterniond& q, const Eigen::Vector3d& v);
    // Eigen::Vector3d passive_quat(const Eigen::Quaterniond& q, const Eigen::Vector3d& v);

    Eigen::Quaterniond chain_quat_post(const std::vector<Eigen::Quaterniond>& q_list);
    Eigen::Quaterniond chain_quat_pre(const std::vector<Eigen::Quaterniond>& q_list);

    // Internal
    Eigen::Matrix4d _make_HR_rotate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d);
    Eigen::Matrix4d _make_HR_translate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d);
    Eigen::Quaterniond normalize_and_canonicalize(Eigen::Quaterniond q);

    Eigen::Vector3d C_to_eul_extr(const Eigen::Matrix3d& C, const std::string& order);
    Eigen::Vector3d C_to_eul_intr(const Eigen::Matrix3d& C, const std::string& order);
    Eigen::Matrix3d eul_to_C_extr(double a, double b, double c, const std::string& order);
    Eigen::Matrix3d eul_to_C_intr(double a, double b, double c, const std::string& order);

    Eigen::Matrix4d make_HC(const Eigen::Matrix3d& C, const Eigen::Vector3d& d, const std::string& first);
    Eigen::Matrix4d _make_HC_translate_first(const Eigen::Matrix3d& C, const Eigen::Vector3d& d);
    Eigen::Matrix4d _make_HC_rotate_first(const Eigen::Matrix3d& C, const Eigen::Vector3d& d);

    Eigen::Matrix3d eul_to_C(double a, double b, double c, const std::string& order, const std::string& type);
    Eigen::Matrix3d eul_to_R(double a, double b, double c, const std::string& order, const std::string& type);
    Eigen::Quaterniond eul_to_quatR(double a, double b, double c, const std::string& order, const std::string& type);
    Eigen::Quaterniond eul_to_quatC(double a, double b, double c, const std::string& order, const std::string& type);


    Eigen::Quaterniond rot_to_quat(const Eigen::Matrix3d& rot);

    Eigen::Vector3d quatC_to_eul(const Eigen::Quaterniond& qC, const std::string& order, const std::string& type);
    Eigen::Vector3d C_to_eul(const Eigen::Matrix3d& C, const std::string& order, const std::string& type);



}


