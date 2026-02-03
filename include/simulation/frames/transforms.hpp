#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

namespace transforms {

    double deg2rad(double deg);
    double rad2deg(double rad);

    /**
    * @brief Returns a rotation matrix about the "x" axis
    *
    * @details Given an "angle" in degrees,
            this function returns a rotation matrix "Rx" that encodes a rotation about the "x" axis of a coordinate frame by "angle" degrees
    *
    * @param angle Angle to rotate by [degrees]
    *
    * @return Rotation matrix "Rx" that encodes a rotation about the 'x' axis of a coordinate frame by "angle" degrees
    */
    Eigen::Matrix3d Rx(double angle);



    /**
    * @brief Returns a rotation matrix about the "y" axis
    *
    * @details Given an "angle" in degrees,
            this function returns a rotation matrix "Ry" that encodes a rotation about the "y" axis of a coordinate frame by "angle" degrees
    *
    * @param angle Angle to rotate by [degrees]
    *
    * @return Rotation matrix "Ry" that encodes a rotation about the "y" axis of a coordinate frame by "angle" degrees
    */
    Eigen::Matrix3d Ry(double angle);



    /**
    * @brief Returns a rotation matrix about the "z" axis
    *
    * @details Given an "angle" in degrees,
            this function returns a rotation matrix "Rz" that encodes a rotation about the "z" axis of a coordinate frame by "angle" degrees
    *
    * @param angle Angle to rotate by [degrees]
    *
    * @return Rotation matrix "Rz" that encodes a rotation about the "z" axis of a coordinate frame by "angle" degrees
    */
    Eigen::Matrix3d Rz(double angle);



    Eigen::Matrix3d eul2rotm_extr(double roll, double pitch, double yaw, const std::string& order);
    Eigen::Matrix3d eul2rotm_intr(double roll, double pitch, double yaw, const std::string& order);

    Eigen::Vector3d rotm2eul_extr(const Eigen::Matrix3d& R, const std::string& order);
    Eigen::Vector3d rotm2eul_intr(const Eigen::Matrix3d& R, const std::string& order);


    /**
    * @brief Performs an active rotation
    *
    * @details Given vector “v0” expressed WRT frame {0} and a rotation matrix "R" that rotates vector "v0" into vector "v1" WRT frame {0}, 
            this function returns vector "v1" expressed WRT frame {0}
    *
    * @param R Rotation matrix that rotates vector v0 into v1
    * @param v0 Vector "v0" expressed WRT frame {0}
    *
    * @return Vector "v1" expressed WRT frame {0}
    */
    Eigen::Vector3d active_rot(const Eigen::Matrix3d& R, const Eigen::Vector3d& v0);



    /**
    * @brief Performs a passive rotation
    *
    * @details Given vector “v” expressed WRT frame {0} and a rotation matrix "R" that rotates frame {0} into frame {1}, 
            this function returns vector "v" expressed WRT frame {1}
    *
    * @param R Rotation matrix that rotates frame {0} into frame {1}
    * @param v Vector "v" expressed WRT frame {0}
    *
    * @return Vector "v" expressed WRT frame {1}
    */
    Eigen::Vector3d passive_rot(const Eigen::Matrix3d& R, const Eigen::Vector3d& v);



    /**
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
    Eigen::Matrix4d makeH(const Eigen::Matrix3d& R, const Eigen::Vector3d& d, const std::string& first);



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



    /**
    * @brief Performs an active homogenous transformation
    *
    * @details Given vector “v0” expressed WRT frame {0} and a homogenous transformation matrix "H" that transforms vector "v0" into vector "v1" WRT frame {0}, 
            this function returns vector "v1" expressed WRT frame {0}
    *
    * @param H Homogenous transformation matrix that transforms vector v0 into v1
    * @param v0 Vector "v0" expressed WRT frame {0}
    *
    * @return Vector "v1" expressed WRT frame {0}
    */
    Eigen::Vector3d active_hom(const Eigen::Matrix4d& H, const Eigen::Vector3d& v);



    /**
    * @brief Performs a passive homogenous transformation
    *
    * @details Given vector “p” expressed WRT frame {0} and a homogenous transformation matrix "H" that transforms frame {0} into frame {1}, 
            this function returns vector "p" expressed WRT frame {1}
    *
    * @param H Homogenous transformation matrix that transforms frame {0} into frame {1}
    * @param v Vector "v" expressed WRT frame {0}
    *
    * @return Vector "v" expressed WRT frame {1}
    */
    Eigen::Vector3d passive_hom(const Eigen::Matrix4d& H, const Eigen::Vector3d& v);

    Eigen::Matrix4d chain_hom_intr(const std::vector<Eigen::Matrix4d>& H_list);

    Eigen::Matrix4d chain_hom_extr(const std::vector<Eigen::Matrix4d>& H_list);

    Eigen::Matrix3d chain_rot_intr(const std::vector<Eigen::Matrix3d>& R_list);

    Eigen::Matrix3d chain_rot_extr(const std::vector<Eigen::Matrix3d>& R_list);

    Eigen::Quaterniond eul2quat_extr(double a, double b, double c, const std::string& order);

    Eigen::Quaterniond eul2quat_intr(double a, double b, double c, const std::string& order);

    Eigen::Quaterniond qx(double phi);
    Eigen::Quaterniond qy(double theta);
    Eigen::Quaterniond qz(double psi);

    Eigen::Quaterniond quat_mul(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);


    Eigen::Quaterniond eul2quat_extr(double a, double b, double c, const std::string& order);

    Eigen::Quaterniond eul2quat_intr(double a, double b, double c, const std::string& order);

    Eigen::Matrix3d quat2rot(const Eigen::Quaterniond& q_in);

    Eigen::Vector3d quat2eul_extr(const Eigen::Quaterniond& q, const std::string& order);
    Eigen::Vector3d quat2eul_intr(const Eigen::Quaterniond& q, const std::string& order);

    Eigen::Vector3d active_quat(const Eigen::Quaterniond& q, const Eigen::Vector3d& v);
    Eigen::Vector3d passive_quat(const Eigen::Quaterniond& q, const Eigen::Vector3d& v);


    Eigen::Quaterniond chain_quat_intr(const std::vector<Eigen::Quaterniond>& q_list);

    Eigen::Quaterniond chain_quat_extr(const std::vector<Eigen::Quaterniond>& q_list);


    // Internal 
    Eigen::Matrix4d _identity_hom();
    Eigen::Matrix4d _make_H_rotate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d);
    Eigen::Matrix4d _make_H_translate_first(const Eigen::Matrix3d& R, const Eigen::Vector3d& d);
    Eigen::Quaterniond _normalize_and_canonicalize(Eigen::Quaterniond q);

}