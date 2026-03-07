#include <iostream>
#include <cmath>
#include <Eigen/Dense>


static double wrap_pi(double a){
    while (a >  M_PI) a -= 2.0*M_PI;
    while (a < -M_PI) a += 2.0*M_PI;
    return a;
}

static double yaw_from_CIB(const Eigen::Matrix3d& CIB){
    // ZYX yaw extraction (valid for general attitude, stable when pitch not near +/- 90deg)
    return std::atan2(CIB(1,0), CIB(0,0));
}

// ------------------------------------------------------------
// Case 1: Stationary spin about x, cancel gravity in BODY
//
// Intent:
//   - Set w_B = (30 deg/s, 0, 0)
//   - Apply force to cancel gravity in BODY so no net translation ideally
//   - Moment = 0 so w should remain constant (spherical J)
//
// Expected (ideal):
//   - No translation drift (p ~ constant, v ~ 0)
//   - Roll changes by 180 deg over 6 s (30 deg/s * 6 s = 180 deg)
// ------------------------------------------------------------
void case1(vehicles::Aircraft& plane){

    Eigen::Vector3d gB = plane.BODY.gB.data;
    dynamics::Force FB_net{ -gB };                       // cancel gravity in BODY
    dynamics::Moment Mnet_B{ Eigen::Vector3d(0, 0, 0) }; // no moments

    plane.BODY.wB_BN.data = Eigen::Vector3d(global::deg2rad(30), 0, 0);

    dynamics::RigidBodyState xt = plane.BODY.RigidBodyState();

    int tf = static_cast<int>(6.0 / dynamics::common::dt);
    for (int t = 0; t < tf; ++t){
        xt = dynamics::step_rigid_body(xt, plane.properties.mass, plane.properties.J, FB_net, Mnet_B);
    }

    std::cout << "\n--- Case 1 complete ---\n";
}

// ------------------------------------------------------------
// Case 2: Pure translation, constant force, no rotation
//
// Setup:
//   m = 1 kg
//   F_B = (1,0,0) N constant
//   M_B = (0,0,0)
//   w_B = (0,0,0)
//   v_B(0) = (0,0,0)
//   p(0) = (0,0,0)
//   run for T = 2 s
//
// Expected analytic results:
//
//   a_x = F_x/m = 1 m/s^2
//
//   v_x(T) = a_x*T = 2 m/s
//   v_y(T) = 0
//   v_z(T) = 0
//
//   p_x(T) = (1/2)*a_x*T^2 = 2 m
//   p_y(T) = 0
//   p_z(T) = 0
//
//   attitude unchanged, quaternion stays identity
// ------------------------------------------------------------
void case2(vehicles::Aircraft& plane){

    // Reset initial conditions
    plane.BODY.wB_BN.data = Eigen::Vector3d(0,0,0);
    plane.BODY.v.data = Eigen::Vector3d(0,0,0);

    dynamics::Force  FB_net{ Eigen::Vector3d(1.0, 0.0, 0.0) };
    dynamics::Moment Mnet_B{ Eigen::Vector3d(0.0, 0.0, 0.0) };

    dynamics::RigidBodyState xt = plane.BODY.RigidBodyState();

    const double T = 2.0;
    int tf = static_cast<int>(T / dynamics::common::dt);
    for (int t = 0; t < tf; ++t){
        xt = dynamics::step_rigid_body(xt, plane.properties.mass, plane.properties.J, FB_net, Mnet_B);
    }

    // Print actual vs expected
    const double a = FB_net.data.x() / plane.properties.mass.data; // 1.0
    const double vx_exp = a * T;                                   // 2.0
    const double px_exp = 0.5 * a * T * T;                         // 2.0

    std::cout << "\n--- Case 2 ---\n";
    std::cout << "Expected v_x = " << vx_exp << " m/s, Actual v_x = " << xt.v.data.x() << " m/s\n";
    std::cout << "Expected p_x = " << px_exp << " m,   Actual p_x = " << xt.p.data.x() << " m\n";
}

// ------------------------------------------------------------
// Case 3: Pure rotation, no forces/moments, constant body rate about z
//
// Setup:
//   J = I (identity inertia)
//   F_B = (0,0,0)
//   M_B = (0,0,0)
//   w_B = (0,0, 90 deg/s)
//   v_B(0) = (0,0,0)
//   p(0) = (0,0,0)
//   run for T = 1 s
//
// Expected analytic results:
//
//   Rotational dynamics:
//     w_dot = 0  (since M=0 and J spherical)
//     w_z(T) = w_z(0) = 90 deg/s = pi/2 rad/s
//
//   Quaternion kinematics yields total yaw change:
//     psi(T) = w_z * T = 90 deg = pi/2 rad
//
//   Translation:
//     v_B(T) = (0,0,0)
//     p(T) unchanged
// ------------------------------------------------------------
void case3(vehicles::Aircraft& plane){

    plane.BODY.wB_BN.data = Eigen::Vector3d(0, 0, global::deg2rad(90.0));
    plane.BODY.v.data = Eigen::Vector3d(0, 0, 0);

    dynamics::Force  FB_net{ Eigen::Vector3d(0.0, 0.0, 0.0) };
    dynamics::Moment Mnet_B{ Eigen::Vector3d(0.0, 0.0, 0.0) };

    dynamics::RigidBodyState xt = plane.BODY.RigidBodyState();

    const double T = 1.0;
    int tf = static_cast<int>(T / dynamics::common::dt);
    for (int t = 0; t < tf; ++t){
        xt = dynamics::step_rigid_body(xt, plane.properties.mass, plane.properties.J, FB_net, Mnet_B);
    }

    // Extract yaw from quaternion result for comparison
    const Eigen::Matrix3d CIB = transforms::quat2rot(xt.q.data);
    const double yaw_act = wrap_pi(yaw_from_CIB(CIB));
    const double yaw_exp = wrap_pi(global::deg2rad(90.0) * T);

    std::cout << "\n--- Case 3 ---\n";
    std::cout << "Expected yaw = " << yaw_exp << " rad, Actual yaw = " << yaw_act << " rad\n";
    std::cout << "Expected yaw = " << global::rad2deg(yaw_exp) << " deg, Actual yaw = " << global::rad2deg(yaw_act) << " deg\n";
}


int main(){

    // define frames
    frames::ECEFFrame ECEF;
    frames::NEDFrame NED;
    frames::FRDFrame BODY;

    double lat_0 = 0.0, lon_0 = 0.0;
    Eigen::Matrix3d CEN_0 = frames::CEN_from_lat_lon(lat_0, lon_0);
    Eigen::Vector3d pE_NE_0(global::r_earth, 0, 0);
    NED.HEN->set(CEN_0, pE_NE_0);

    Eigen::Matrix3d CNB_0 = global::I3;
    Eigen::Vector3d pN_BN_0(0, 0, 0);
    BODY.HNB->set(CNB_0, pN_BN_0);

    Eigen::Matrix3d CEB_0 = CNB_0 * CEN_0;
    Eigen::Vector3d pE_BE_0 = CEN_0.transpose() * pN_BN_0 + pE_NE_0;
    BODY.HEB->set(CEB_0, pE_BE_0);

    Eigen::Vector3d gE = frames::common::gECEF(pE_BE_0);
    BODY.gB = dynamics::Gravity{ CEB_0 * gE };

    // create vehicle
    dynamics::Mass mass { 1.0 }; // kg
    Eigen::Matrix3d J { Eigen::Matrix3d::Identity() };
    vehicles::Properties properties{ .mass = mass, .J = J };
    vehicles::Aircraft plane{ .BODY = BODY, .properties = properties };

    // run cases
    case1(plane);
    case2(plane);
    case3(plane);

    return 0;
}
