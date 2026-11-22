#include "simulation/guidance/public/manager.hpp"

#include <stdexcept>

namespace guidance
{

	GuidanceManagerOutput GuidanceManager::step(
		const GuidanceManagerInput& input)
	{
		if (regulation.has_value()) {
			return {.setpoint = (*regulation).step()};
		}
		if (tracking.has_value()) {
			return {.setpoint = (*tracking).step()};
		}
		if (interpolated.has_value()) {
			return {.setpoint = (*interpolated).step(input.kf)};
		}
		throw std::runtime_error("control::step invalid trajectory type");
	}
} // namespace guidance
