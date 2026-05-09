#pragma once

namespace aerodynamics {

    struct FreeStreamVelocity {
        double data;
    };

    struct AngleOfAttack {
        double data;
    };

    struct SideslipAngle {
        double data;
    };

    struct AerodynamicState {
        FreeStreamVelocity Vinf;
        AngleOfAttack alpha;
        SideslipAngle beta;
    };

    template <typename T>
    struct AerodynamicState_T {
        T Vinf = T(0);
        T alpha = T(0);
        T beta = T(0);
    };

}
