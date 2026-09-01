#pragma once
#include "simulation/constants/public.hpp"
#include "simulation/dynamics/public/data/types.hpp"

namespace geography {

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

#include "simulation/geography/public/detail/gravity.tpp"
