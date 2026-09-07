#include "simulation/aerodynamics/public/components/surface.hpp"

namespace aerodynamics
{

	void Surface::compute_geometry()
	{
		area = chord * span;
		AR = span / chord;
	}

} // namespace aerodynamics
