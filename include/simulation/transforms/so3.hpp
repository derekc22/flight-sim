#pragma once
#include <Eigen/Dense>

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

    Eigen::Matrix3d _eul_to_R_extr(double roll, double pitch, double yaw, const std::string& order);
    Eigen::Matrix3d _eul_to_R_intr(double roll, double pitch, double yaw, const std::string& order);

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

   
    Eigen::Matrix3d C_from_R(const Eigen::Matrix3d& R);
    Eigen::Matrix3d R_from_C(const Eigen::Matrix3d& C);

    Eigen::Matrix3d chain_rot_post(const std::vector<Eigen::Matrix3d>& rot_list);
    Eigen::Matrix3d chain_rot_pre(const std::vector<Eigen::Matrix3d>& rot_list);

    Eigen::Vector3d _C_to_eul_extr(const Eigen::Matrix3d& C, const std::string& order);
    Eigen::Vector3d _C_to_eul_intr(const Eigen::Matrix3d& C, const std::string& order);
    Eigen::Matrix3d _eul_to_C_extr(double a, double b, double c, const std::string& order);
    Eigen::Matrix3d _eul_to_C_intr(double a, double b, double c, const std::string& order);

    Eigen::Matrix3d eul_to_C(double a, double b, double c, const std::string& order, const std::string& type);
    Eigen::Matrix3d eul_to_R(double a, double b, double c, const std::string& order, const std::string& type);
    Eigen::Quaterniond rot_to_quat(const Eigen::Matrix3d& rot);

    Eigen::Vector3d C_to_eul(const Eigen::Matrix3d& C, const std::string& order, const std::string& type);



}


