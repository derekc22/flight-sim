#include <Eigen/Dense>
#include <vector>
#include <string>
#include <unordered_map>
#include <optional>
#include <algorithm>
#include <cmath>
#include "simulation/actuators/surface/shared.hpp"
#include "simulation/aerodynamics/aerodynamics.hpp"
#include "simulation/constants/constants.hpp"
#include "simulation/structural/structural.hpp"
#include "simulation/util/util.hpp"

namespace aerodynamics {

    AerodynamicProperties::AerodynamicProperties(std::vector<Surface> s) : surfaces(std::move(s)) {
        compute_aero_properties();
        surfaceIDs = build_IDs();
    }

    void AerodynamicProperties::compute_aero_properties() {
        for (Surface& s : surfaces) {
            s.area = s.chord * s.span;
            s.AR   = s.span / s.chord;
            s.p_ac = s.p_ref;   // quarter-chord assumed already in p_ref
        }
    }

    std::unordered_map<std::string, size_t> AerodynamicProperties::build_IDs() {
        std::unordered_map<std::string, size_t> m;
        for (size_t i = 0; i < surfaces.size(); ++i) {
            m[surfaces[i].id] = i;
        }
        return m;
    }

};
