#pragma once
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public.hpp"
#include "simulation/frames/public.hpp"

namespace geography {

    struct Latitude {
        double data;
    };

    struct Longitude {
        double data;
    };

    struct Altitude {
        double data;
    };

    struct Heading {
        double data;
    };

    struct GeographicState {
        Latitude lat;
        Longitude lon;
        Altitude alt;
    };

    /** @warning The parent of F must the ECEFFrame */
    GeographicState compute_geographic_state(const frames::Frame& F);

    dynamics::OrientationMatrix CEN_from_lat_lon(const geography::Latitude& lat, const geography::Longitude& lon);
    dynamics::Position pE_from_lat_lon_alt(const geography::GeographicState& geographic_state);

    dynamics::Gravity gN();

    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::HomogeneousTransformationMatrix& HEB);
    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::OrientationMatrix& CEB);
    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::OrientationQuaternion& qEB);
    dynamics::Gravity gB(const dynamics::Position& pE, const dynamics::EulerAngles& eulEB);

    dynamics::Gravity gB(const dynamics::HomogeneousTransformationMatrix& HNB);
    dynamics::Gravity gB(const dynamics::OrientationMatrix& CNB);
    dynamics::Gravity gB(const dynamics::OrientationQuaternion& qNB);
    dynamics::Gravity gB(const dynamics::EulerAngles& eulNB);

    dynamics::Gravity gS(const dynamics::Gravity& gB, const dynamics::OrientationMatrix& CBS);
    dynamics::Gravity gW(const dynamics::Gravity& gS, const dynamics::OrientationMatrix& CSW);

    template <typename T>
    constants::Vector3_T<T> gB_T(const T& phi, const T& theta);

}
#include "simulation/geography/public.tpp"
