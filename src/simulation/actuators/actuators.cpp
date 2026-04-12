#include "simulation/actuators/actuators.hpp"
#include "simulation/util/util.hpp"
#include "simulation/control/control.hpp"

namespace actuators {

        double Actuator::_step(double cmd, std::optional<double>& lag_state) {
            double cmd_clamped = util::clamp(cmd, limit_min, limit_max); // clamp
            double prev_cmd = lag_state ? lag_state.value() : cmd_clamped;
            double cmd_lagged = util::first_order_lag(cmd_clamped, prev_cmd, tau); // apply EMA
            lag_state = cmd_lagged;
            return cmd_lagged;
        }

        control::ControlSurfaceInputs ActuatorProperties::step(const control::ControlSurfaceInputs& u_cmd){
            return {
                .elevator = actuators.elevator._step(u_cmd.elevator, actuators.elevator.prev_cmd),
                .aileron = actuators.aileron._step(u_cmd.aileron, actuators.aileron.prev_cmd),
                .rudder = actuators.rudder._step(u_cmd.rudder, actuators.rudder.prev_cmd),
                .flaps = actuators.flaps._step(u_cmd.flaps, actuators.flaps.prev_cmd),
                .spoilers = actuators.spoilers._step(u_cmd.spoilers, actuators.spoilers.prev_cmd)
            };
        }

}
