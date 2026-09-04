
#include "simulation/vehicles/public/aircraft.hpp"

#include "simulation/aerodynamics/public/data/helpers.hpp"
#include "simulation/constants/public/linalg.hpp"
#include "simulation/frames/public/detail/kinematics.hpp"
#include "simulation/geography/public/detail/geodesy.hpp"
#include "simulation/geography/public/detail/gravity.hpp"

#include <queue>
#include <stdexcept>
#include <string>
#include <unordered_set>

namespace vehicles
{

	void Aircraft::step(
		const StepOptions& opts)
	{

		StepOptions::validate(opts);

		if (opts.NEDFrameECEFStepOpts.has_value()) {
			if (!stepped_NEDFrameECEF) {
				_StepOptions _opts;
				_opts.lat = opts.NEDFrameECEFStepOpts->lat_NE;
				_opts.lon = opts.NEDFrameECEFStepOpts->lon_NE;
				_opts.alt = opts.NEDFrameECEFStepOpts->alt_NE;
				_opts.geo = opts.NEDFrameECEFStepOpts->geo_NE;

				_StepOptions::validate(_opts);
				Aircraft::step(NEDFrameECEF, _opts);
				Aircraft::step_dependents(NEDFrameECEF);

				stepped_NEDFrameECEF = true;
			} else {
				std::string err_msg = "vehicles::Aircraft::step: Error. Attempting to re-step NEDFrameECEF";
				throw std::invalid_argument(err_msg);
			}
		}

		if (opts.FRDFrameECEFStepOpts.has_value()) {
			_StepOptions _opts;
			_opts.H = opts.FRDFrameECEFStepOpts->HEB;
			_opts.C = opts.FRDFrameECEFStepOpts->CEB;
			_opts.p = opts.FRDFrameECEFStepOpts->pE_BE;
			_opts.q = opts.FRDFrameECEFStepOpts->qEB;
			_opts.eul = opts.FRDFrameECEFStepOpts->eulEB;
			_opts.C_dot = opts.FRDFrameECEFStepOpts->CEB_dot;
			_opts.q_dot = opts.FRDFrameECEFStepOpts->qEB_dot;
			_opts.w = opts.FRDFrameECEFStepOpts->wB_BE;
			_opts.eul_dot = opts.FRDFrameECEFStepOpts->eulEB_dot;
			_opts.wq = opts.FRDFrameECEFStepOpts->wq_BE;
			_opts.v = opts.FRDFrameECEFStepOpts->vB_BE;
			_opts.lat = opts.FRDFrameECEFStepOpts->lat_BE;
			_opts.lon = opts.FRDFrameECEFStepOpts->lon_BE;
			_opts.alt = opts.FRDFrameECEFStepOpts->alt_BE;
			_opts.X = opts.FRDFrameECEFStepOpts->X_BE;
			_opts.geo = opts.FRDFrameECEFStepOpts->geo_BE;

			_StepOptions::validate(_opts);
			Aircraft::step(FRDFrameECEF, _opts);
			Aircraft::step_dependents(FRDFrameECEF);
		}

		if (opts.FRDFrameNEDStepOpts.has_value()) {
			_StepOptions _opts;
			_opts.H = opts.FRDFrameNEDStepOpts->HNB;
			_opts.C = opts.FRDFrameNEDStepOpts->CNB;
			_opts.p = opts.FRDFrameNEDStepOpts->pN_BN;
			_opts.q = opts.FRDFrameNEDStepOpts->qNB;
			_opts.eul = opts.FRDFrameNEDStepOpts->eulNB;
			_opts.C_dot = opts.FRDFrameNEDStepOpts->CNB_dot;
			_opts.q_dot = opts.FRDFrameNEDStepOpts->qNB_dot;
			_opts.w = opts.FRDFrameNEDStepOpts->wB_BN;
			_opts.eul_dot = opts.FRDFrameNEDStepOpts->eulNB_dot;
			_opts.wq = opts.FRDFrameNEDStepOpts->wq_BN;
			_opts.v = opts.FRDFrameNEDStepOpts->vB_BN;
			_opts.X = opts.FRDFrameNEDStepOpts->X_BN;

			_StepOptions::validate(_opts);
			Aircraft::step(FRDFrameNED, _opts);
			Aircraft::step_dependents(FRDFrameNED);
		}

		if (opts.CGFrameFRDStepOpts.has_value()) {
			_StepOptions _opts;
			_opts.p = opts.CGFrameFRDStepOpts->pB_GB;
			_opts.X = opts.CGFrameFRDStepOpts->X_GB;

			_StepOptions::validate(_opts);
			Aircraft::step(CGFrameFRD, _opts);
			Aircraft::step_dependents(CGFrameFRD);
		}

		if (opts.STABFrameFRDStepOpts.has_value()) {
			_StepOptions _opts;
			_opts.alpha = opts.STABFrameFRDStepOpts->alpha;
			_opts.aero = opts.STABFrameFRDStepOpts->aero;

			_StepOptions::validate(_opts);
			Aircraft::step(STABFrameFRD, _opts);
			Aircraft::step_dependents(STABFrameFRD);
		}

		if (opts.WINDFrameSTABStepOpts.has_value()) {
			_StepOptions _opts;
			_opts.beta = opts.WINDFrameSTABStepOpts->beta;
			_opts.aero = opts.WINDFrameSTABStepOpts->aero;

			_StepOptions::validate(_opts);
			Aircraft::step(WINDFrameSTAB, _opts);
			Aircraft::step_dependents(WINDFrameSTAB);
		}

		// Sync all gravity vectors
		step_gravity();
	}

	void Aircraft::step(
		frames::NEDFrameECEF& F,
		const _StepOptions& opts)
	{

		bool recursive = !opts;
		frames::SetOptions NEDFrameECEFSetOpts;

		if (!recursive) {
			// final guard
			if (opts.H.has_value() || opts.C.has_value() || opts.p.has_value() || opts.q.has_value() ||
				opts.eul.has_value() || opts.C_dot.has_value() || opts.q_dot.has_value() || opts.w.has_value() ||
				opts.eul_dot.has_value() || opts.wq.has_value() || opts.v.has_value() || opts.g.has_value()) {
				std::string err_msg = "vehicles::Aircraft::_step: Invalid _StepOptions input passed for NEDFrameECEF";
				throw std::invalid_argument(err_msg);
			}

			if (opts.lat.has_value() && opts.lon.has_value() && opts.alt.has_value()) {
				NEDFrameECEFSetOpts.p =
					geography::pE_from_lat_lon_alt(geography::GeographicState{*opts.lat, *opts.lon, *opts.alt});
				NEDFrameECEFSetOpts.C = geography::CEN_from_lat_lon(*opts.lat, *opts.lon);
			}

			if (opts.geo.has_value()) {
				NEDFrameECEFSetOpts.p = geography::pE_from_lat_lon_alt(*opts.geo);
				NEDFrameECEFSetOpts.C = geography::CEN_from_lat_lon(opts.geo->lat, opts.geo->lon);
			}
		}

		else {
			std::string err_msg = "vehicles::Aircraft::_step: NEDFrameECEF is not a dependent to any step-able frame. "
								  "It can never be recursively stepped. Something has gone wrong.";
			throw std::invalid_argument(err_msg);
		}

		// Invariants
		NEDFrameECEFSetOpts.C_dot = dynamics::OrientationMatrixRate{constants::Zero3x3};
		NEDFrameECEFSetOpts.v = dynamics::TranslationalVelocity{constants::Zero3};

		F.set(NEDFrameECEFSetOpts);
	}

	void Aircraft::step(
		frames::FRDFrameECEF& F,
		const _StepOptions& opts)
	{

		bool recursive = !opts;
		frames::SetOptions FRDFrameECEFSetOpts;

		if (!recursive) {

			// final guard
			if (opts.g.has_value()) {
				std::string err_msg = "vehicles::Aircraft::_step: Invalid _StepOptions input passed for FRDFrameECEF";
				throw std::invalid_argument(err_msg);
			}

			if (opts.H.has_value()) {
				FRDFrameECEFSetOpts.H = *opts.H;
			}

			if (opts.C.has_value()) {
				FRDFrameECEFSetOpts.C = *opts.C;
			}

			if (opts.p.has_value()) {
				FRDFrameECEFSetOpts.p = *opts.p;
			}

			if (opts.q.has_value()) {
				FRDFrameECEFSetOpts.q = *opts.q;
			}

			if (opts.eul.has_value()) {
				FRDFrameECEFSetOpts.eul = *opts.eul;
			}

			if (opts.C_dot.has_value()) {
				FRDFrameECEFSetOpts.C_dot = *opts.C_dot;
			}

			if (opts.q_dot.has_value()) {
				FRDFrameECEFSetOpts.q_dot = *opts.q_dot;
			}

			if (opts.w.has_value()) {
				FRDFrameECEFSetOpts.w = *opts.w;
			}

			if (opts.eul_dot.has_value()) {
				FRDFrameECEFSetOpts.eul_dot = *opts.eul_dot;
			}

			if (opts.wq.has_value()) {
				FRDFrameECEFSetOpts.wq = *opts.wq;
			}

			if (opts.v.has_value()) {
				FRDFrameECEFSetOpts.v = *opts.v;
			}

			if (opts.lat.has_value() && opts.lon.has_value() && opts.alt.has_value()) {
				dynamics::Position p =
					geography::pE_from_lat_lon_alt(geography::GeographicState{*opts.lat, *opts.lon, *opts.alt});
				FRDFrameECEFSetOpts.p = p;
			}

			if (opts.X.has_value()) {
				FRDFrameECEFSetOpts.p = opts.X->p;
				FRDFrameECEFSetOpts.q = opts.X->q;
				FRDFrameECEFSetOpts.v = opts.X->v;
				FRDFrameECEFSetOpts.w = opts.X->w;
			}

			if (opts.geo.has_value()) {
				dynamics::Position p = geography::pE_from_lat_lon_alt(*opts.geo);
				FRDFrameECEFSetOpts.p = p;
			}
		}

		else {
			dynamics::Position p{frames::transform_point(FRDFrameNED.HNB.p().data, NEDFrameECEF, ECEFFrame)};
			FRDFrameECEFSetOpts.p = p;
			dynamics::OrientationMatrix C{FRDFrameNED.HNB.C().data * NEDFrameECEF.HEN.C().data};
			FRDFrameECEFSetOpts.C = C;
			FRDFrameECEFSetOpts.w = FRDFrameNED.wB_BN;
			FRDFrameECEFSetOpts.v = FRDFrameNED.vB_BN;
		}

		F.set(FRDFrameECEFSetOpts);
	}

	void Aircraft::step(
		frames::FRDFrameNED& F,
		const _StepOptions& opts)
	{

		bool recursive = !opts;
		frames::SetOptions FRDFrameNEDSetOpts;

		if (!recursive) {

			// final guard
			if (opts.g.has_value()) {
				std::string err_msg = "vehicles::Aircraft::_step: Invalid _StepOptions input passed for FRDFrameNED";
				throw std::invalid_argument(err_msg);
			}

			if (opts.H.has_value()) {
				FRDFrameNEDSetOpts.H = *opts.H;
			}

			if (opts.C.has_value()) {
				FRDFrameNEDSetOpts.C = *opts.C;
			}

			if (opts.p.has_value()) {
				FRDFrameNEDSetOpts.p = *opts.p;
			}

			if (opts.q.has_value()) {
				FRDFrameNEDSetOpts.q = *opts.q;
			}

			if (opts.eul.has_value()) {
				FRDFrameNEDSetOpts.eul = *opts.eul;
			}

			if (opts.C_dot.has_value()) {
				FRDFrameNEDSetOpts.C_dot = *opts.C_dot;
			}

			if (opts.q_dot.has_value()) {
				FRDFrameNEDSetOpts.q_dot = *opts.q_dot;
			}

			if (opts.w.has_value()) {
				FRDFrameNEDSetOpts.w = *opts.w;
			}

			if (opts.eul_dot.has_value()) {
				FRDFrameNEDSetOpts.eul_dot = *opts.eul_dot;
			}

			if (opts.wq.has_value()) {
				FRDFrameNEDSetOpts.wq = *opts.wq;
			}

			if (opts.v.has_value()) {
				FRDFrameNEDSetOpts.v = *opts.v;
			}

			if (opts.X.has_value()) {
				FRDFrameNEDSetOpts.p = opts.X->p;
				FRDFrameNEDSetOpts.q = opts.X->q;
				FRDFrameNEDSetOpts.v = opts.X->v;
				FRDFrameNEDSetOpts.w = opts.X->w;
			}
		}

		else {
			FRDFrameNEDSetOpts.p =
				dynamics::Position{frames::transform_point(FRDFrameECEF.HEB.p().data, ECEFFrame, NEDFrameECEF)};
			dynamics::OrientationMatrix C{FRDFrameECEF.HEB.C().data * NEDFrameECEF.HEN.C().data.transpose()};
			FRDFrameNEDSetOpts.C = C;
			FRDFrameNEDSetOpts.w = FRDFrameECEF.wB_BE;
			FRDFrameNEDSetOpts.v = FRDFrameECEF.vB_BE;
		}

		F.set(FRDFrameNEDSetOpts);
	}

	void Aircraft::step(
		frames::CGFrameFRD& F,
		const _StepOptions& opts)
	{

		bool recursive = !opts;
		frames::SetOptions CGFrameFRDSetOpts;

		if (!recursive) {

			// final guard
			if (opts.H.has_value() || opts.C.has_value() || opts.q.has_value() || opts.eul.has_value() ||
				opts.C_dot.has_value() || opts.q_dot.has_value() || opts.w.has_value() || opts.eul_dot.has_value() ||
				opts.wq.has_value() || opts.v.has_value() || opts.g.has_value()) {
				std::string err_msg = "vehicles::Aircraft::_step: Invalid _StepOptions input passed for CGFrameFRD";
				throw std::invalid_argument(err_msg);
			}

			if (opts.p.has_value()) {
				CGFrameFRDSetOpts.p = *opts.p;
			}

			if (opts.X.has_value()) {
				CGFrameFRDSetOpts.p = opts.X->p;
				CGFrameFRDSetOpts.q = opts.X->q;
				CGFrameFRDSetOpts.v = opts.X->v;
				CGFrameFRDSetOpts.w = opts.X->w;
			}
		}

		else {
			// By design, recursive updates use only information available from the frame hierarchy
			// All non-frame information must enter explicitly through StepOptions
			// Updating CGFrameFRD requires information that cannot be derived from the frame hierarchy alone: CG location
			// Therefore, recursive stepping leaves this frame unchanged
		}

		// Invariants
		CGFrameFRDSetOpts.C = dynamics::OrientationMatrix{constants::I3};
		CGFrameFRDSetOpts.C_dot = dynamics::OrientationMatrixRate{constants::Zero3x3};
		CGFrameFRDSetOpts.v = dynamics::TranslationalVelocity{constants::Zero3};

		F.set(CGFrameFRDSetOpts);
	}

	void Aircraft::step(
		frames::STABFrameFRD& F,
		const _StepOptions& opts)
	{

		bool recursive = !opts;
		frames::SetOptions STABFrameFRDSetOpts;

		if (!recursive) {

			// final guard
			if (opts.H.has_value() || opts.C.has_value() || opts.p.has_value() || opts.q.has_value() ||
				opts.eul.has_value() || opts.C_dot.has_value() || opts.q_dot.has_value() || opts.w.has_value() ||
				opts.eul_dot.has_value() || opts.wq.has_value() || opts.v.has_value() || opts.g.has_value()) {
				std::string err_msg = "vehicles::Aircraft::_step: Invalid _StepOptions input passed for STABFrameFRD";
				throw std::invalid_argument(err_msg);
			}

			if (opts.alpha.has_value()) {
				dynamics::OrientationMatrix C = aerodynamics::CBS(*opts.alpha);
				STABFrameFRDSetOpts.C = C;
			}

			if (opts.aero.has_value()) {
				dynamics::OrientationMatrix C = aerodynamics::CBS(opts.aero->alpha);
				STABFrameFRDSetOpts.C = C;
			}
		}

		else {
			// By design, recursive updates use only information available from the frame hierarchy
			// All non-frame information must enter explicitly through StepOptions
			// Updating STABFrameFRD requires information that cannot be derived from the frame hierarchy alone: alpha
			// Therefore, recursive stepping leaves this frame unchanged
		}

		// Invariants
		STABFrameFRDSetOpts.p = dynamics::Position{constants::Zero3};
		STABFrameFRDSetOpts.C_dot = dynamics::OrientationMatrixRate{constants::Zero3x3};
		STABFrameFRDSetOpts.v = dynamics::TranslationalVelocity{constants::Zero3};

		F.set(STABFrameFRDSetOpts);
	}

	void Aircraft::step(
		frames::WINDFrameSTAB& F,
		const _StepOptions& opts)
	{

		bool recursive = !opts;
		frames::SetOptions WINDFrameSTABSetOpts;

		if (!recursive) {

			// final guard
			if (opts.H.has_value() || opts.C.has_value() || opts.p.has_value() || opts.q.has_value() ||
				opts.eul.has_value() || opts.C_dot.has_value() || opts.q_dot.has_value() || opts.w.has_value() ||
				opts.eul_dot.has_value() || opts.wq.has_value() || opts.v.has_value() || opts.g.has_value()) {
				std::string err_msg = "vehicles::Aircraft::_step: Invalid _StepOptions input passed for WINDFrameSTAB";
				throw std::invalid_argument(err_msg);
			}

			if (opts.beta.has_value()) {
				dynamics::OrientationMatrix C = aerodynamics::CSW(*opts.beta);
				WINDFrameSTABSetOpts.C = C;
			}

			if (opts.aero.has_value()) {
				dynamics::OrientationMatrix C = aerodynamics::CSW(opts.aero->beta);
				WINDFrameSTABSetOpts.C = C;
			}
		}

		else {
			// By design, recursive updates use only information available from the frame hierarchy
			// All non-frame information must enter explicitly through StepOptions
			// Updating WINDFrameSTAB requires information that cannot be derived from the frame hierarchy alone: beta
			// Therefore, recursive stepping leaves this frame unchanged
		}

		// Invariants
		WINDFrameSTABSetOpts.p = dynamics::Position{constants::Zero3};
		WINDFrameSTABSetOpts.C_dot = dynamics::OrientationMatrixRate{constants::Zero3x3};
		WINDFrameSTABSetOpts.v = dynamics::TranslationalVelocity{constants::Zero3};

		F.set(WINDFrameSTABSetOpts);
	}

	void Aircraft::step_dependents(
		frames::Frame& root)
	{
		std::queue<frames::Frame*> q;
		std::unordered_set<frames::Frame*> visited;

		q.push(&root);
		visited.insert(&root);

		while (!q.empty()) {
			frames::Frame* curr = q.front();
			q.pop();

			if (curr != &root) {
				step_dependent(*curr);
			}

			for (frames::Frame* dep : curr->dependents) {
				// if a dependent has not already been visited, mark it as visited and add it to the queue
				if (visited.insert(dep).second) {
					q.push(dep);
				}
			}
		}
	}

	void Aircraft::step_dependent(
		frames::Frame& F)
	{
		if (F.id == frames::FrameID::NEDFrameECEF)
			return step(static_cast<frames::NEDFrameECEF&>(F), {});
		if (F.id == frames::FrameID::FRDFrameECEF)
			return step(static_cast<frames::FRDFrameECEF&>(F), {});
		if (F.id == frames::FrameID::FRDFrameNED)
			return step(static_cast<frames::FRDFrameNED&>(F), {});
		if (F.id == frames::FrameID::CGFrameFRD)
			return step(static_cast<frames::CGFrameFRD&>(F), {});
		if (F.id == frames::FrameID::STABFrameFRD)
			return step(static_cast<frames::STABFrameFRD&>(F), {});
		if (F.id == frames::FrameID::WINDFrameSTAB)
			return step(static_cast<frames::WINDFrameSTAB&>(F), {});

		std::string err_msg =
			"vehicles::Aircraft::step_dependent: Attempting to recursively step an unknown frame type";
		throw std::invalid_argument(err_msg);
	}

	void Aircraft::step_gravity()
	{

		// Sync NEDFrameECEF gravity
		NEDFrameECEF.gN = geography::gN();

		// Sync FRDFrameECEF gravity
		FRDFrameECEF.gB = geography::gB(FRDFrameECEF.HEB.p(), FRDFrameECEF.HEB.C());

		// Sync FRDFrameNED gravity
		// Two options for setting FRDFrameNED.gB
		// Option A: FRDFrameNED.gB = FRDFrameECEF.gB
		// Ensures gravity in the FRDFrameNED is always synced with the true ECEF‑based gravity vector at the aircraft's location
		// Option B: FRDFrameNED.gB = geography::gB(FRDFrameNED.HNB.C())
		// Performs a coordinate transform into the body frame of NEDFrameECEF's gravity vector
		// Because NEDFrameECEF is fixed at the initialization point and is never updated, this approximation slowly drifts from the true gravity vector (FRDFrameECEF.gB) as the aircraft moves away from the NED origin
		// Option A: best for global consistency of FRDFrameNED.gB with FRDFrameECEF.gB. Both frames agree on the same gravity vector derived from the true ECEF position
		// Option B: best for local congruence of FRDFrameNED with the fixed NEDFrameECEF frame, but FRDFrameNED.gB will diverge from the true ECEF‑based gravity vector (FRDFrameECEF.gB) as the aircraft moves
		// Verdict: For maximum calculation consistency, we choose option A
		FRDFrameNED.gB = FRDFrameECEF.gB;
		// FRDFrameNED.gB = geography::gB(FRDFrameNED.HNB.C());

		// Sync CGFrameFRD gravity
		CGFrameFRD.gG = FRDFrameNED.gB;

		// Sync STABFrameFRD and WINDFrameSTAB gravity
		STABFrameFRD.gS = geography::gS(FRDFrameNED.gB, STABFrameFRD.HBS.C());
		WINDFrameSTAB.gW = geography::gW(STABFrameFRD.gS, WINDFrameSTAB.HSW.C());
	}

	void Aircraft::init_frames()
	{

		// Set default values
		frames::SetOptions init_step_options;
		init_step_options.H = dynamics::HomogeneousTransformationMatrix{constants::HI};
		init_step_options.w = dynamics::AngularVelocity{constants::Zero3};
		init_step_options.v = dynamics::TranslationalVelocity{constants::Zero3};
		init_step_options.g = dynamics::Gravity{constants::Zero3};

		NEDFrameECEF.set(init_step_options);
		FRDFrameECEF.set(init_step_options);
		FRDFrameNED.set(init_step_options);
		CGFrameFRD.set(init_step_options);
		STABFrameFRD.set(init_step_options);
		WINDFrameSTAB.set(init_step_options);

		// Set dependents, {X} -depends-on-> {Y}
		NEDFrameECEF.add_as_direct_dependent(&FRDFrameNED); // {FRDFrameNED} -> {NEDFrameECEF}

		FRDFrameECEF.add_as_direct_dependent(&FRDFrameNED); // {FRDFrameNED} -> {FRDFrameECEF}

		FRDFrameNED.add_as_direct_dependent(&FRDFrameECEF); // {FRDFrameECEF} -> {FRDFrameNED}
		FRDFrameNED.add_as_direct_dependent(&CGFrameFRD);	// {CGFrameFRD} -> {FRDFrameNED}
		FRDFrameNED.add_as_direct_dependent(&STABFrameFRD); // {STABFrameFRD} -> {FRDFrameNED}

		STABFrameFRD.add_as_direct_dependent(&WINDFrameSTAB); // {WINDFrameSTAB} -> {STABFrameFRD}
	}

	Aircraft::Aircraft(
		const std::string& id,
		const structural::StructuralManager& structural_manager,
		const aerodynamics::AerodynamicsManager& aerodynamics_manager,
		const actuators::ActuatorManager& actuator_manager,
		const control::ControlManager& control_manager,
		const sensors::SensorManager& sensor_manager,
		const avionics::AvionicsManager& avionics_manager,
		const guidance::GuidanceManager& guidance_manager,
		const estimation::EstimationManager& estimation_manager,
		const allocator::AllocatorManager& allocator_manager)
		: id(id),

		  ECEFFrame{},
		  NEDFrameECEF{&ECEFFrame},
		  FRDFrameECEF{&ECEFFrame},
		  FRDFrameNED{&NEDFrameECEF},
		  CGFrameFRD{&FRDFrameNED},
		  STABFrameFRD{&FRDFrameNED},
		  WINDFrameSTAB{&STABFrameFRD},

		  structural_manager(structural_manager),
		  aerodynamics_manager(aerodynamics_manager),
		  actuator_manager(actuator_manager),
		  propulsion_manager{},
		  control_manager(control_manager),
		  sensor_manager(sensor_manager),
		  avionics_manager(avionics_manager),
		  guidance_manager(guidance_manager),
		  estimation_manager(estimation_manager),
		  allocator_manager(allocator_manager)
	{
		init_frames();
	}

} // namespace vehicles
