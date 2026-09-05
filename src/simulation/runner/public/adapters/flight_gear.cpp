#include "simulation/runner/public/adapters/flight_gear.hpp"

#include "simulation/constants/public/linalg.hpp"
#include "simulation/frames/public/detail/kinematics.hpp"
#include "simulation/vehicles/public/aircraft.hpp"

namespace runner
{

	FlightGearAdapter::FlightGearAdapter() : udp_out(5510), udp_in("127.0.0.1", 5511) {}

	FlightGearAdapterOutput FlightGearAdapter::receive(
		const FlightGearAdapterInput& input)
	{
		// fetch from FlightGear
		if (auto out_msg = udp_out.try_receive()) {
			cached_msg_out = messages::process_out_msg(out_msg.value());
		}

		// apply wind
		atmospheric::Wind windI{constants::Zero3};
		atmospheric::Wind windB{constants::Zero3};
		if (input.wind_flag) {
			windI.data = cached_msg_out.wind.data;
			windB.data = frames::transform_vec(windI.data, input.aircraft.NEDFrameECEF, input.aircraft.FRDFrameNED);
		}

		return {.windI = windI, .windB = windB};
	}

	void FlightGearAdapter::send(
		const FlightGearAdapterSendInput& input)
	{
		// generate in_msg from the simulation state
		messages::FlightGearMessageIn in_msg = messages::process_in_msg(input.geo_t1, input.eulNB);

		// send message
		udp_in.send(in_msg);
	}

} // namespace runner
