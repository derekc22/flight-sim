#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include "simulation/dynamics/shared.hpp"
#include "simulation/geography/shared.hpp"

namespace frames { struct Frame; } // forward declare

namespace geography {

    /** @warning The parent of F must the ECEFFrame */
    GeographicState compute_geographic_state(const frames::Frame& F);

    dynamics::OrientationMatrix CEN_from_lat_lon(const geography::Latitude& lat, const geography::Longitude& lon);

    geography::GeographicState lat_lon_alt_from_pE(const dynamics::Position & pE);

    dynamics::Position pE_from_lat_lon_alt(const geography::GeographicState& geographic_state);

    /** @brief Given the position of an object WRT the ECEF frame, expressed in the ECEF frame (pE), this function returns the local gravity vector at that position expressed in the ECEF frame (gE) 
        Get local ECEF gravity vector expressed in the ECEF frame based on your current position WRT the ECEF frame expressed in the ECEF frame
    */
    dynamics::Gravity gE(const dynamics::Position& pE);

    dynamics::Gravity gN();

    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::HomogenousFrameTransformationMatrix& HEB); // pE is unused but left for congruence
    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::OrientationMatrix& CEB);
    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::OrientationQuaternion& qEB);
    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::EulerAngles& eulEB);
    
    dynamics::Gravity gB(const dynamics::HomogenousFrameTransformationMatrix& HNB);
    dynamics::Gravity gB(const dynamics::OrientationMatrix& CNB);
    dynamics::Gravity gB(const dynamics::OrientationQuaternion& qNB);
    dynamics::Gravity gB(const dynamics::EulerAngles& eulNB);

    dynamics::Gravity gS(const dynamics::Gravity& gB, const dynamics::OrientationMatrix& CBS);
    dynamics::Gravity gW(const dynamics::Gravity& gS, const dynamics::OrientationMatrix& CSW);

}
