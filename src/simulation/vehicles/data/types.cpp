#include "simulation/vehicles/public/data/types.hpp"

#include <stdexcept>
#include <string>

namespace vehicles
{

	void StepOptions::validate(
		const StepOptions& opts)
	{
		std::string err_msg = "vehicles::StepOptions::validate: Invalid input, cannot pass StepOptions for "
							  "FRDFrameECEF and FRDFrameNED simultaneously";
		if (opts.FRDFrameECEFStepOpts.has_value() && opts.FRDFrameNEDStepOpts.has_value())
			throw std::invalid_argument(err_msg);
	}

	void _StepOptions::validate(
		const _StepOptions& opts)
	{

		const bool has_H = opts.H.has_value();
		const bool has_C = opts.C.has_value();
		const bool has_p = opts.p.has_value();
		const bool has_q = opts.q.has_value();
		const bool has_eul = opts.eul.has_value();
		const bool has_C_dot = opts.C_dot.has_value();
		const bool has_q_dot = opts.q_dot.has_value();
		const bool has_w = opts.w.has_value();
		const bool has_eul_dot = opts.eul_dot.has_value();
		const bool has_wq = opts.wq.has_value();
		const bool has_v = opts.v.has_value();
		const bool has_g = opts.g.has_value();
		const bool has_lat = opts.lat.has_value();
		const bool has_lon = opts.lon.has_value();
		const bool has_alt = opts.alt.has_value();
		const bool has_alpha = opts.alpha.has_value();
		const bool has_beta = opts.beta.has_value();
		const bool has_X = opts.X.has_value();
		const bool has_geo = opts.geo.has_value();
		const bool has_aero = opts.aero.has_value();

		// lat, lon, alt must be passed together
		const bool has_geo_any = (has_lat || has_lon || has_alt);
		const bool has_geo_all = (has_lat && has_lon && has_alt);

		if (has_geo_any && !has_geo_all) {
			std::string err_msg =
				"vehicles::_StepOptions::validate: Invalid geographic input, lat, lon, alt must be passed together";
			throw std::invalid_argument(err_msg);
		}

		// H cannot be combined with any other position or orientation representations
		if (has_H && (has_C || has_p || has_q || has_eul || has_geo_any || has_geo)) {
			std::string err_msg = "vehicles::_StepOptions::validate: Invalid position or orientation input, cannot "
								  "pass C, p, q, eul, (lat, lon, alt), geo with H";
			throw std::invalid_argument(err_msg);
		}

		// Only one position representation at a time
		if ((has_p && has_geo_any) || (has_p && has_geo) || (has_geo_any && has_geo)) {
			std::string err_msg =
				"vehicles::_StepOptions::validate: Invalid position input, pass at most one of p, (lat, lon, alt), geo";
			throw std::invalid_argument(err_msg);
		}

		// Only one orientation representation at a time
		if ((has_C && has_q) || (has_C && has_eul) || (has_q && has_eul)) {
			std::string err_msg =
				"vehicles::_StepOptions::validate: Invalid orientation input, pass at most one of C, q, eul";
			throw std::invalid_argument(err_msg);
		}

		// Only one attitude-rate representation at a time
		if ((has_C_dot && (has_q_dot || has_w || has_eul_dot || has_wq)) ||
			(has_q_dot && (has_w || has_eul_dot || has_wq)) || (has_w && (has_eul_dot || has_wq)) ||
			(has_eul_dot && has_wq)) {
			std::string err_msg = "vehicles::_StepOptions::validate: Invalid attitude-rate input, pass at most one of "
								  "C_dot, q_dot, w, eul_dot, wq";
			throw std::invalid_argument(err_msg);
		}

		// RigidBodyState cannot be combined with any other state representations
		if (has_X &&
			(has_H || has_C || has_p || has_q || has_eul || has_C_dot || has_q_dot || has_w || has_eul_dot || has_wq ||
				has_v || has_geo_any || has_geo)) {
			std::string err_msg = "vehicles::_StepOptions::validate: Invalid state input, cannot pass H, C, p, q, eul, "
								  "(lat, lon, alt), geo, C_dot, q_dot, w, eul_dot, wq with X";
			throw std::invalid_argument(err_msg);
		}

		// Only one aerodynamics representation at a time
		if (has_aero && (has_alpha || has_beta)) {
			std::string err_msg =
				"vehicles::_StepOptions::validate: Invalid aerodynamics input, cannot pass alpha, beta with aero";
			throw std::invalid_argument(err_msg);
		}
	}

	_StepOptions::operator bool() const
	{
		return H.has_value() || C.has_value() || p.has_value() || q.has_value() || eul.has_value() ||
			C_dot.has_value() || q_dot.has_value() || w.has_value() || eul_dot.has_value() || wq.has_value() ||
			v.has_value() || g.has_value() || lat.has_value() || lon.has_value() || alt.has_value() ||
			alpha.has_value() || beta.has_value() || X.has_value() || geo.has_value() || aero.has_value();
	}

} // namespace vehicles
