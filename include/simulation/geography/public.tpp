#pragma once
#include "simulation/util/cppad/public.hpp"
#include "simulation/util/trig/public.hpp"

namespace geography {

    template <typename T>
    constants::Vector3_T<T> gB_T(const T& phi, const T& theta) {
        constants::Vector3_T<T> gB;
        gB << -T(constants::g_earth) * util::sin(theta),
               T(constants::g_earth) * util::sin(phi) * util::cos(theta),
               T(constants::g_earth) * util::cos(phi) * util::cos(theta);
        return gB;
    }

}
