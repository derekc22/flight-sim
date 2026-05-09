#pragma once

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

}
