#pragma once
#include <complex>
#include <string>
#include "analysis/eigenanalysis/public.hpp"

namespace analysis {

	std::string format_complex(const std::complex<double>& z, int precision = 6);

}
