#pragma once
#include "simulation/propulsion/public/components/effector.hpp"

namespace propulsion
{

	struct PropulsorEffectors {
		PropulsorEffector front_propulsor;
		PropulsorEffector left_propulsor;
		PropulsorEffector right_propulsor;
	};

} // namespace propulsion
