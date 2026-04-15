#pragma once

namespace control { struct ControlLawParameters; struct AxisControlLawInput; struct FullStateControlLawInput;} // forward declare

namespace control {

    struct PIDController {
        double Kp = 0;
        double Kd = 0;
        double Ki = 0;

        double integral = 0.0;
        double d_filtered = 0.0;
        double prev_err = 0.0;
        double tau;

        PIDController(const ControlLawParameters& params);

        double _step(const AxisControlLawInput& ctrl_law_input);
    };

    struct RollPIDController : PIDController {
        RollPIDController(const ControlLawParameters& params);
    };

    struct PitchPIDController : PIDController {
        PitchPIDController(const ControlLawParameters& params);
    };

    struct YawPIDController : PIDController {
        YawPIDController(const ControlLawParameters& params);
    };

    struct RollDamper : PIDController {
        RollDamper(const ControlLawParameters& params);
    };

    struct PitchDamper : PIDController {
        PitchDamper(const ControlLawParameters& params);
    };

    struct YawDamper : PIDController {
        YawDamper(const ControlLawParameters& params);
    };

}
