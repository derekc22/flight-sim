#include <complex>
#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include "simulation/analysis/private.hpp"

namespace analysis {

    std::string format_complex(const std::complex<double>& z, int precision) {
        std::ostringstream out;
        out << std::fixed << std::setprecision(precision) << z.real();
        if (z.imag() < 0.0) out << " - " << std::abs(z.imag()) << "i";
        else out << " + " << z.imag() << "i";
        return out.str();
    }

}
