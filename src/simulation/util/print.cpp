#include <spdlog/spdlog.h>
#include <sstream>
#include "simulation/constants/public/scalars.hpp"
#include "simulation/util/public/print.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/geography/public/detail/gravity.hpp"
#include "simulation/aerodynamics/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/util/public/trig.hpp"

namespace util {

	std::string print_vec(
	    const char* name,
	    const Eigen::Vector3d& x,
	    const char* unit)
	{
		std::ostringstream ss;
		ss << std::fixed << std::setprecision(3);

		ss << std::left << std::setw(8) << name << "[ " << std::right << std::setw(10) << x.x() << ", " << std::right
		   << std::setw(10) << x.y() << ", " << std::right << std::setw(10) << x.z() << " ] " << unit << '\n';

		return ss.str();
	}

	void print_state(
	    int t,
	    const dynamics::RigidBodyState& Xt,
	    const geography::GeographicState& geo,
	    const aerodynamics::AerodynamicState& aero,
	    const atmospheric::Wind& windI)
	{

		const Eigen::Vector3d& p = Xt.p.data;

		dynamics::EulerAngles eul;
		eul.set(Xt.q);

		const Eigen::Vector3d& v = Xt.v.data;
		const Eigen::Vector3d& w = Xt.w.data;
		const Eigen::Vector3d& g = geography::gB(Xt.q).data;
		const Eigen::Vector3d& wind = windI.data;

		std::ostringstream ss;
		ss << std::fixed << std::setprecision(3);

		ss << "\n"
		   << "t       " << t * constants::dt << " [s]\n"
		   << "---------------------------------------------------------------------------------\n";

		ss << print_vec("p", p, "[m]");

		ss << std::left << std::setw(8) << "eul"
		   << "[ " << std::right << std::setw(10) << util::rad_to_deg(eul.psi()) << ", " << std::right << std::setw(10)
		   << util::rad_to_deg(eul.theta()) << ", " << std::right << std::setw(10) << util::rad_to_deg(eul.phi())
		   << " ] [deg]\n";

		ss << print_vec("v", v, "[m/s]");

		ss << std::left << std::setw(8) << "w"
		   << "[ " << std::right << std::setw(10) << util::rad_to_deg(w.x()) << ", " << std::right << std::setw(10)
		   << util::rad_to_deg(w.y()) << ", " << std::right << std::setw(10) << util::rad_to_deg(w.z())
		   << " ] [deg/s]\n";

		ss << print_vec("g", g, "[m/s^2]");
		ss << print_vec("wind", wind, "[m/s]");

		ss << std::left << std::setw(8) << "geo"
		   << "lat: " << std::right << std::setw(10) << util::rad_to_deg(geo.lat.data) << " [deg], "
		   << "lon: " << std::right << std::setw(10) << util::rad_to_deg(geo.lon.data) << " [deg], "
		   << "alt: " << std::right << std::setw(10) << geo.alt.data << " [m]\n";

		ss << std::left << std::setw(8) << "aero"
		   << "alpha: " << std::right << std::setw(10) << util::rad_to_deg(aero.alpha.data) << " [deg], "
		   << "beta: " << std::right << std::setw(10) << util::rad_to_deg(aero.beta.data) << " [deg]\n";

		ss << "---------------------------------------------------------------------------------\n";

		spdlog::info(ss.str());
	}

} // namespace util
