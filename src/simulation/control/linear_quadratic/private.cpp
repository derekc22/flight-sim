#include "simulation/control/linear_quadratic/private.hpp"

namespace control {

    ControlOutput make_control_output(const actuators::ActuatorInputsVector_T<double>& u_cmd) {
        actuators::SurfaceActuatorInputs_T<double> u_surface{};
        actuators::PropulsorActuatorInputs_T<double> u_propulsor{};

        u_surface.elevator_cmd = u_cmd[0];
        u_surface.aileron_cmd = u_cmd[1];
        u_surface.rudder_cmd = u_cmd[2];

        u_propulsor.front_propulsor_cmd = u_cmd[3];
        u_propulsor.left_propulsor_cmd = u_cmd[4];
        u_propulsor.right_propulsor_cmd = u_cmd[5];

        return { u_surface, u_propulsor };
    }

}
