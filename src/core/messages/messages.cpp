#include "include/messages/messages.hpp"

namespace messages {

Vector3D::Vector3D() : x(0), y(0), z(0) {};
Vector3D::Vector3D(double x, double y, double z) : x(x), y(y), z(z) {};

Quaternion::Quaternion() : w(1), x(0), y(0), z(0) {};
Quaternion::Quaternion(double w, double x, double y, double z) : w(w), x(x), y(y), z(z) {};

DroneState::DroneState(Vector3D pos, Vector3D vel, Quaternion att, Vector3D omg) : pos(pos), vel(vel), att(att), omg(omg) {};

}
