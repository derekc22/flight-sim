#pragma once
#include <string>
#include "simulation/actuators/public/manager.hpp"
#include "simulation/aerodynamics/public/manager.hpp"
#include "simulation/allocator/public/manager.hpp"
#include "simulation/avionics/public/manager.hpp"
#include "simulation/control/public/manager.hpp"
#include "simulation/estimation/public/manager.hpp"
#include "simulation/frames/public/frame.hpp"
#include "simulation/guidance/public/manager.hpp"
#include "simulation/propulsion/public/manager.hpp"
#include "simulation/sensors/public/manager.hpp"
#include "simulation/structural/public/manager.hpp"
#include "simulation/vehicles/public/data/types.hpp"

namespace vehicles {

	struct Aircraft {
		std::string id;

		frames::ECEFFrame ECEFFrame;
		frames::NEDFrameECEF NEDFrameECEF;
		frames::FRDFrameNED FRDFrameNED;
		frames::FRDFrameECEF FRDFrameECEF;
		frames::CGFrameFRD CGFrameFRD;
		frames::STABFrameFRD STABFrameFRD;
		frames::WINDFrameSTAB WINDFrameSTAB;

		bool stepped_NEDFrameECEF = false;

		structural::StructuralManager structural_manager;
		aerodynamics::AerodynamicsManager aerodynamics_manager;
		actuators::ActuatorManager actuator_manager;
		propulsion::PropulsionManager propulsion_manager;
		control::ControlManager control_manager;
		sensors::SensorManager sensor_manager;
		avionics::AvionicsManager avionics_manager;
		guidance::GuidanceManager guidance_manager;
		estimation::EstimationManager estimation_manager;
		allocator::AllocatorManager allocator_manager;

		Aircraft(const std::string& id,
		    const structural::StructuralManager& structural_manager,
		    const aerodynamics::AerodynamicsManager& aerodynamics_manager,
		    const actuators::ActuatorManager& actuator_manager,
		    const control::ControlManager& control_manager,
		    const sensors::SensorManager& sensor_manager,
		    const avionics::AvionicsManager& avionics_manager,
		    const guidance::GuidanceManager& guidance_manager,
		    const estimation::EstimationManager& estimation_manager,
		    const allocator::AllocatorManager& allocator_manager);

		void step(const StepOptions& opts);

		void step(frames::NEDFrameECEF& F, const _StepOptions& opts);
		void step(frames::FRDFrameNED& F, const _StepOptions& opts);
		void step(frames::FRDFrameECEF& F, const _StepOptions& opts);
		void step(frames::CGFrameFRD& F, const _StepOptions& opts);
		void step(frames::STABFrameFRD& F, const _StepOptions& opts);
		void step(frames::WINDFrameSTAB& F, const _StepOptions& opts);

		void step_dependents(frames::Frame& root);
		void step_dependent(frames::Frame& F);

		void step_gravity();
		void init_frames();
	};

} // namespace vehicles
