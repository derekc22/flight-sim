#pragma once
#include <Eigen/Dense>

namespace transforms {

    Eigen::Quaterniond _eul_to_quatR_extr(double a, double b, double c, const std::string& order);
    Eigen::Quaterniond _eul_to_quatR_intr(double a, double b, double c, const std::string& order);
    Eigen::Quaterniond _eul_to_quatC_extr(double a, double b, double c, const std::string& order);
    Eigen::Quaterniond _eul_to_quatC_intr(double a, double b, double c, const std::string& order);

    Eigen::Quaterniond qx(double phi);
    Eigen::Quaterniond qy(double theta);
    Eigen::Quaterniond qz(double psi);

    // Eigen::Quaterniond quat_mul(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);


    Eigen::Matrix3d quat_to_rot(const Eigen::Quaterniond& q);

    Eigen::Vector3d quatR_to_eul_extr(const Eigen::Quaterniond& q, const std::string& order);
    Eigen::Vector3d quatR_to_eul_intr(const Eigen::Quaterniond& q, const std::string& order);
    Eigen::Vector3d _quatC_to_eul_extr(const Eigen::Quaterniond& q, const std::string& order);
    Eigen::Vector3d _quatC_to_eul_intr(const Eigen::Quaterniond& q, const std::string& order);

    // Eigen::Vector3d active_quat(const Eigen::Quaterniond& q, const Eigen::Vector3d& v);
    // Eigen::Vector3d passive_quat(const Eigen::Quaterniond& q, const Eigen::Vector3d& v);

    Eigen::Quaterniond chain_quat_post(const std::vector<Eigen::Quaterniond>& q_list);
    Eigen::Quaterniond chain_quat_pre(const std::vector<Eigen::Quaterniond>& q_list);
    Eigen::Quaterniond normalize_and_canonicalize(Eigen::Quaterniond q);

    Eigen::Quaterniond eul_to_quatR(double a, double b, double c, const std::string& order, const std::string& type);
    Eigen::Quaterniond eul_to_quatC(double a, double b, double c, const std::string& order, const std::string& type);



    Eigen::Vector3d quatC_to_eul(const Eigen::Quaterniond& qC, const std::string& order, const std::string& type);
}


