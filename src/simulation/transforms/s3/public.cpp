#include <Eigen/Dense>
#include <stdexcept>
#include <algorithm>
#include <vector>
#include "simulation/constants/public.hpp"
#include "simulation/transforms/s3/public.hpp"
#include "simulation/transforms/s3/s3.hpp"
#include "simulation/transforms/so3/public.hpp"
#include "simulation/util/public.hpp"

namespace transforms {

    Eigen::Quaterniond normalize_and_canonicalize(Eigen::Quaterniond q) {
        q.normalize();
        // Canonicalize sign: q and -q represent the same rotation
        if (q.w() < 0.0) q.coeffs() *= -1.0; // coeffs() is (x,y,z,w)
        return q;
    }

    Eigen::Quaterniond eul_to_quatR(double a, double b, double c, const std::string& order, const std::string& type){
        if (type == "extr") return eul_to_quatR_extr(a, b, c, order);
        if (type == "intr") return eul_to_quatR_intr(a, b, c, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }

    Eigen::Quaterniond eul_to_quatC(double a, double b, double c, const std::string& order, const std::string& type){
        if (type == "extr") return eul_to_quatC_extr(a, b, c, order);
        if (type == "intr") return eul_to_quatC_intr(a, b, c, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }

    Eigen::Vector3d quatC_to_eul(const Eigen::Quaterniond& qC, const std::string& order, const std::string& type) {
        if (type == "extr") return quatC_to_eul_extr(qC, order);
        if (type == "intr") return quatC_to_eul_intr(qC, order);
        throw std::invalid_argument("Unsupported type: " + type);
    }

    Eigen::Matrix3d quat_to_rot(const Eigen::Quaterniond& q) {
        return Eigen::Matrix3d(transforms::normalize_and_canonicalize(q));
    }



}
