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

    struct GeometricAltitude {
        // current: radial height from Earth center minus spherical Earth radius
        // planned: geodetic/ellipsoidal height above WGS84
        double data;
    };

    struct LocalHeight {
        // positive-up height relative to local NED origin, i.e. -pN_BN.z()
        double data;
    };

    struct PressureAltitude {
        // altitude inferred from static pressure
        double data;
    };

    struct GroundElevation {
        // terrain/ground height relative to the Earth surface
        double data;
    };

    struct HeightAGL {
        // height above the terrain/ground
        double data;
    };
    struct Heading {
        double data;
    };

    struct GeographicState {
        Latitude lat;
        Longitude lon;
        GeometricAltitude alt;
    };

    GeographicState compute_geographic_state(const frames::Frame& F, const frames::Frame& E);

    dynamics::OrientationMatrix CEN_from_lat_lon(const Latitude& lat, const Longitude& lon);

    dynamics::Position pE_from_lat_lon_alt(const GeographicState& geo);

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
