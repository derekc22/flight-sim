#pragma once

namespace propulsion {

    template <typename T>
    struct PropulsorOmegaDot_T {
        T front_propulsor = T(0);
        T left_propulsor = T(0);
        T right_propulsor = T(0);
    };

}
