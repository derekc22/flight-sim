#include "simulation/constants/public.hpp"
#include "simulation/frames/public/detail/kinematics.hpp"
#include "simulation/runner/public/components/flight_gear.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

namespace runner {

    FlightGear::FlightGear() : udp_out(5510), udp_in("127.0.0.1", 5511) {}

    FlightGearOutput FlightGear::receive(const FlightGearInput& input) {
        // fetch from FlightGear
        if (auto out_msg = udp_out.try_receive()) {
            cached_msg_out = messages::process_out_msg(out_msg.value());
        }

        // apply wind
        atmospheric::Wind windI { constants::Zero3 };
        atmospheric::Wind windB { constants::Zero3 };
        if (input.wind_enabled) {
            windI.data = cached_msg_out.wind.data;
            windB.data = frames::transform_vec(
                windI.data,
                input.aircraft.NEDFrameECEF,
                input.aircraft.FRDFrameNED
            );
        }

        return {
            .windI = windI,
            .windB = windB
        };
    }

    void FlightGear::send(const FlightGearSendInput& input) {
        // generate in_msg from the simulation state
        messages::FlightGearMessageIn in_msg = messages::process_in_msg(
            input.geographic_state,
            input.attitude
        );

        // send message
        udp_in.send(in_msg);
    }

}
