#pragma once
#include "simulation/actuators/public/components/surface/aileron.hpp"
#include "simulation/actuators/public/components/surface/elevator.hpp"
#include "simulation/actuators/public/components/surface/flap.hpp"
#include "simulation/actuators/public/components/propulsor/front.hpp"
#include "simulation/actuators/public/components/propulsor/left.hpp"
#include "simulation/actuators/public/components/propulsor/right.hpp"
#include "simulation/actuators/public/components/surface/rudder.hpp"
#include "simulation/actuators/public/components/surface/spoiler.hpp"

namespace actuators {

	struct SurfaceActuators {
		ElevatorActuator elevator;
		AileronActuator aileron;
		RudderActuator rudder;
		FlapActuator flap;
		SpoilerActuator spoiler;
	};

	struct PropulsorActuators {
		FrontPropulsorActuator front_propulsor;
		LeftPropulsorActuator left_propulsor;
		RightPropulsorActuator right_propulsor;
	};

} // namespace actuators
