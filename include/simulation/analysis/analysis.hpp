#pragma once
#include <complex>
#include <string>
#include "simulation/analysis/public.hpp"

namespace analysis {

    std::string format_complex(const std::complex<double>& z, int precision = 6);

}
