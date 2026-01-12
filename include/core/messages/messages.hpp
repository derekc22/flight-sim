#pragma once

namespace messages {

class Vector3D {
    public:

    double x;
    double y;
    double z;

    Vector3D();
    Vector3D(double x, double y, double z);
};

class Quaternion {
    public:

    double w;
    double x;
    double y;
    double z;

    Quaternion();
    Quaternion(double w, double x, double y, double z);
};

class DroneState {
    public:

    Vector3D pos;
    Vector3D vel;
    Quaternion att;
    Vector3D omg;

    DroneState(Vector3D pos, Vector3D vel, Quaternion att, Vector3D omg);
};
}