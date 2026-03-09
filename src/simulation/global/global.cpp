#include "simulation/global/global.hpp"

namespace global {

    // Constants
    const double r_earth = 6.371e6; // [m]
    const double gravity = 9.81;    // [ms^-2]
    const double pi = 3.1415926535897932384626433832795028841971693993751;
    const double eps = 1e-12;
    const double dt = 0.02;        // [s]

    // Special Matrices
    const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    const Eigen::Matrix4d I4 = Eigen::Matrix4d::Identity();
    const Eigen::Matrix4d HI = Eigen::Matrix4d::Identity(); // Identity homogenous transformation matrix
    const Eigen::Matrix3d Zero3x3 = Eigen::Matrix3d::Zero();

    // Special Vectors
    const Eigen::Vector3d Zero3 = Eigen::Vector3d::Zero();
    const Eigen::Vector3d i = Eigen::Vector3d(1, 0, 0);
    const Eigen::Vector3d j = Eigen::Vector3d(0, 1, 0);
    const Eigen::Vector3d k = Eigen::Vector3d(0, 0, 1);


    // Trigonometry
    double deg_to_rad(double deg){
        return deg * (pi / 180.0);
    };

    double rad_to_deg(double rad){
        return rad * (180.0 / pi);
    };

    double sec(double theta) {
        return 1.0 / std::cos(theta);
    }

    double csc(double theta) {
        return 1.0 / std::sin(theta);
    }

    double cot(double theta) {
        return 1.0 / std::tan(theta);
    }

    double kts_to_ms(double kts) {
        return kts/1.94384;
    }

    double ms_to_kts(double ms) {
        return ms * 1.94384;
    }

    double ft_to_m(double ft){
        return ft/3.281;
    }

    double m_to_ft(double m){
        return m * 3.281;
    }


    // Linear Algebra Operators
    Eigen::Matrix3d hat(const Eigen::Vector3d& v){
        Eigen::Matrix3d S;
        S <<     0, -v.z(),   v.y(),
             v.z(),      0,  -v.x(),
            -v.y(),  v.x(),       0;
        return S;
    }

    Eigen::Vector3d vee(const Eigen::Matrix3d& S){
        return Eigen::Vector3d(
            S(2,1),
            S(0,2),
            S(1,0)
        );
    }





}