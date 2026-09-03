#include <sstream>
#include <string>
#include "simulation/linearization/public/data/helpers.hpp"

namespace linearization {

	std::string print_linearization_solution(
	    const LocalLinearization& lin_sol)
	{
		std::ostringstream out;
		out << "A:\n" << lin_sol.A << "\n";
		out << "B:\n" << lin_sol.B << "\n";
		return out.str();
	}

} // namespace linearization
