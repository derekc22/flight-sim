#pragma once
#include "analysis/eigenanalysis/public.hpp"

#include <complex>
#include <string>

namespace analysis
{

	std::string format_complex(const std::complex<double>& z, int precision = 6);

}
