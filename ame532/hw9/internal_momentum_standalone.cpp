#include <iomanip>
#include <iostream>

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
};

Vec3 operator+(Vec3 a, Vec3 b) { return { a.x + b.x, a.y + b.y, a.z + b.z }; }
Vec3 operator-(Vec3 a, Vec3 b) { return { a.x - b.x, a.y - b.y, a.z - b.z }; }
Vec3 operator*(double s, Vec3 v) { return { s * v.x, s * v.y, s * v.z }; }

Vec3 cross(Vec3 a, Vec3 b) {
    return {
        a.y * b.z - a.z * b.y,
        a.z * b.x - a.x * b.z,
        a.x * b.y - a.y * b.x
    };
}

struct Inertia {
    double Jx = 0.20;
    double Jy = 0.30;
    double Jz = 0.40;
};

struct RigidBodyState {
    Vec3 omega_B = {};
};

struct RotorProperties {
    double Jr = 0.02;
    Vec3 axis_B = { 1.0, 0.0, 0.0 };
};

struct RotorState {
    double omega_rel = 0.0;
};

Vec3 mul_diag(const Inertia& J, Vec3 v) { return { J.Jx * v.x, J.Jy * v.y, J.Jz * v.z }; }
Vec3 inv_mul_diag(const Inertia& J, Vec3 v) { return { v.x / J.Jx, v.y / J.Jy, v.z / J.Jz }; }

Vec3 rotor_momentum(const RotorProperties& rotor, const RotorState& rotor_state) {
    // h_int = Jr * omega_rel * axis_B
    return rotor.Jr * rotor_state.omega_rel * rotor.axis_B;
}

Vec3 rotor_reaction_moment(const RotorProperties& rotor, const RotorState& rotor_state, Vec3 omega_B, double omega_rel_dot_cmd) {
    Vec3 h_rotor = rotor_momentum(rotor, rotor_state);
    Vec3 h_rotor_dot = rotor.Jr * omega_rel_dot_cmd * rotor.axis_B;
    // M_int = -(h_int_dot + omega x h_int)
    return (-1.0) * (h_rotor_dot + cross(omega_B, h_rotor));
}

void step_rotor(RotorState& rotor_state, double omega_rel_dot_cmd, double dt) {
    rotor_state.omega_rel += omega_rel_dot_cmd * dt;
}

void step_rigid_body(RigidBodyState& body, const Inertia& J, Vec3 external_moment_B, Vec3 internal_moment_B, double dt) {
    Vec3 H_body = mul_diag(J, body.omega_B);
    // J * omega_dot = M_ext + M_int - omega x (J * omega)
    Vec3 omega_dot_B = inv_mul_diag(J, external_moment_B + internal_moment_B - cross(body.omega_B, H_body));
    body.omega_B = body.omega_B + dt * omega_dot_B;
}

double rotor_accel_command(double t) {
    if (t < 1.0) {
        return 200.0;
    }
    if (t < 2.0) {
        return -200.0;
    }
    return 0.0;
}

int main() {
    // Barebones example:
    // - keep the rigid-body rotational state separate
    // - integrate the internal rotor outside the main 6-DOF state
    // - feed its varying angular momentum back in as an equivalent body moment
    Inertia J;
    RigidBodyState body;
    RotorProperties rotor;
    RotorState rotor_state;

    const double dt = 0.01;
    const double tf = 3.0;

    std::cout << "t,p,q,r,rotor_speed\n";
    for (double t = 0.0; t <= tf; t += dt) {
        double omega_rel_dot_cmd = rotor_accel_command(t);
        Vec3 external_moment_B = {};
        Vec3 internal_moment_B = rotor_reaction_moment(rotor, rotor_state, body.omega_B, omega_rel_dot_cmd);

        step_rigid_body(body, J, external_moment_B, internal_moment_B, dt);
        step_rotor(rotor_state, omega_rel_dot_cmd, dt);

        std::cout << std::fixed << std::setprecision(4)
                  << t << ","
                  << body.omega_B.x << ","
                  << body.omega_B.y << ","
                  << body.omega_B.z << ","
                  << rotor_state.omega_rel << "\n";
    }

    return 0;
}
