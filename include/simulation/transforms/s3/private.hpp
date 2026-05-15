#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>

namespace transforms {

    Eigen::Quaterniond eul_to_quatR_extr(double a, double b, double c, const std::string& order);
    Eigen::Quaterniond eul_to_quatR_intr(double a, double b, double c, const std::string& order);
    Eigen::Quaterniond eul_to_quatC_extr(double a, double b, double c, const std::string& order);
    Eigen::Quaterniond eul_to_quatC_intr(double a, double b, double c, const std::string& order);

    Eigen::Quaterniond qx(double phi);
    Eigen::Quaterniond qy(double theta);
    Eigen::Quaterniond qz(double psi);

    Eigen::Vector3d quatR_to_eul_extr(const Eigen::Quaterniond& q, const std::string& order);
    Eigen::Vector3d quatR_to_eul_intr(const Eigen::Quaterniond& q, const std::string& order);
    Eigen::Vector3d quatC_to_eul_extr(const Eigen::Quaterniond& q, const std::string& order);
    Eigen::Vector3d quatC_to_eul_intr(const Eigen::Quaterniond& q, const std::string& order);

    Eigen::Quaterniond chain_quat_post(const std::vector<Eigen::Quaterniond>& q_list);
    Eigen::Quaterniond chain_quat_pre(const std::vector<Eigen::Quaterniond>& q_list);

	/** @deprecated */
    // Eigen::Quaterniond quat_mul(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2);

}
