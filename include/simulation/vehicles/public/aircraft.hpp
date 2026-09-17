#pragma once
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

#include <string>

namespace vehicles
{

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
			const propulsion::PropulsionManager& propulsion_manager,
			const control::ControlManager& control_manager,
			const sensors::SensorManager& sensor_manager,
			const avionics::AvionicsManager& avionics_manager,
			const guidance::GuidanceManager& guidance_manager,
			const estimation::EstimationManager& estimation_manager,
			const allocator::AllocatorManager& allocator_manager);

		/**
		 * @brief Updates the aircraft frames requested by the step options.
		 *
		 * Each explicit frame update is followed by recursive updates of its
		 * dependents. Gravity is synchronized after all requested updates, and the
		 * fixed NED frame can be explicitly initialized only once.
		 *
		 * @param[in] opts Frame updates to apply.
		 */
		void step(const StepOptions& opts);

		/**
		 * @brief Updates the fixed NED frame relative to ECEF.
		 *
		 * Position and orientation are derived from the geographic representation
		 * in @p opts. Orientation rate and translational velocity are set to zero.
		 *
		 * @param[in,out] F NED frame to update.
		 * @param[in] opts Geographic frame-step data.
		 */
		void step(frames::NEDFrameECEF& F, const _StepOptions& opts);

		/**
		 * @brief Updates the body FRD frame relative to NED.
		 *
		 * Populated options are applied directly. With empty options, position and
		 * orientation are derived from the body FRD frame relative to ECEF and the
		 * fixed NED frame, while angular and translational velocity are copied from
		 * the ECEF-relative body frame.
		 *
		 * @param[in,out] F NED-relative body FRD frame to update.
		 * @param[in] opts Explicit frame-step data, or empty options for a dependent update.
		 */
		void step(frames::FRDFrameNED& F, const _StepOptions& opts);

		/**
		 * @brief Updates the body FRD frame relative to ECEF.
		 *
		 * Populated options are applied directly. With empty options, position and
		 * orientation are derived from the NED-relative body FRD frame and the fixed
		 * NED frame, while angular and translational velocity are copied from the
		 * NED-relative body frame.
		 *
		 * @param[in,out] F ECEF-relative body FRD frame to update.
		 * @param[in] opts Explicit frame-step data, or empty options for a dependent update.
		 */
		void step(frames::FRDFrameECEF& F, const _StepOptions& opts);

		/**
		 * @brief Updates the center-of-gravity frame relative to the body FRD frame.
		 *
		 * Position-only updates use identity orientation and zero angular velocity.
		 * A supplied rigid-body state also applies its orientation and angular
		 * velocity. Empty options retain the existing center-of-gravity position and
		 * use identity orientation and zero angular velocity. Translational velocity
		 * is set to zero for every update.
		 *
		 * @param[in,out] F Center-of-gravity frame to update.
		 * @param[in] opts Explicit center-of-gravity data, or empty options for a dependent update.
		 */
		void step(frames::CGFrameFRD& F, const _StepOptions& opts);

		/**
		 * @brief Updates the stability frame relative to the body FRD frame.
		 *
		 * Explicit angle of attack or aerodynamic-state data determines the
		 * orientation. Empty options retain the existing orientation because angle
		 * of attack cannot be derived from the frame hierarchy. Position,
		 * orientation rate, and translational velocity are set to zero.
		 *
		 * @param[in,out] F Stability frame to update.
		 * @param[in] opts Explicit aerodynamic data, or empty options for a dependent update.
		 */
		void step(frames::STABFrameFRD& F, const _StepOptions& opts);

		/**
		 * @brief Updates the wind frame relative to the stability frame.
		 *
		 * Explicit sideslip angle or aerodynamic-state data determines the
		 * orientation. Empty options retain the existing orientation because
		 * sideslip cannot be derived from the frame hierarchy. Position, orientation
		 * rate, and translational velocity are set to zero.
		 *
		 * @param[in,out] F Wind frame to update.
		 * @param[in] opts Explicit aerodynamic data, or empty options for a dependent update.
		 */
		void step(frames::WINDFrameSTAB& F, const _StepOptions& opts);

		/**
		 * @brief Updates every frame that depends on a root frame.
		 *
		 * Dependents are traversed breadth-first and updated at most once. The root
		 * frame itself is not updated.
		 *
		 * @param[in] root Root of the dependent-frame traversal.
		 */
		void step_dependents(frames::Frame& root);

		/**
		 * @brief Dispatches a dependent update according to the frame type.
		 *
		 * @param[in,out] F Dependent frame to update.
		 */
		void step_dependent(frames::Frame& F);

		/**
		 * @brief Synchronizes gravity across the aircraft frame hierarchy.
		 *
		 * The fixed NED frame receives standard NED gravity. Body gravity is evaluated
		 * from the ECEF-relative aircraft state, shared with the NED-relative body and
		 * center-of-gravity frames, and transformed into the stability and wind frames.
		 */
		void step_gravity();

		/**
		 * @brief Initializes frame state and dependent-frame relationships.
		 *
		 * Modeled frames receive identity transforms and zero angular velocity,
		 * translational velocity, and gravity before the dependency graph is built.
		 */
		void init_frames();
	};

} // namespace vehicles
