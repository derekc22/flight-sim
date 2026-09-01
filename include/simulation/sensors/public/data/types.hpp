#pragma once
#include "simulation/aerodynamics/public/data/types.hpp"
#include "simulation/atmospheric/public/data/types.hpp"
#include "simulation/dynamics/public/data/types.hpp"
#include "simulation/geography/public/data/types.hpp"

namespace sensors {

    // directly measured
    struct AngleOfAttackMeasurement : aerodynamics::AngleOfAttack {};

    struct TranslationalAccelerationMeasurement : dynamics::TranslationalAcceleration {};

    struct AngularVelocityMeasurement : dynamics::AngularVelocity {};

    struct StagnationAirPressureMeasurement : atmospheric::StagnationAirPressure {};

    struct StaticAirPressureMeasurement : atmospheric::StaticAirPressure {};

    struct StagnationAirTemperatureMeasurement : atmospheric::StagnationAirTemperature {};

    struct PositionMeasurement : dynamics::Position {};

    struct TranslationalVelocityMeasurement : dynamics::TranslationalVelocity {};

    struct HeadingMeasurement : geography::Heading {};


    struct SensorGroundTruth {
        const aerodynamics::AngleOfAttack alpha;
        const dynamics::TranslationalAcceleration fB;
        const dynamics::AngularVelocity wB_BI;
        const atmospheric::StagnationAirPressure P0;
        const atmospheric::StaticAirPressure P;
        const atmospheric::StagnationAirTemperature T0;
        const dynamics::Position pI_BI;
        const dynamics::TranslationalVelocity vB_BI;
        const geography::Heading heading;
    };

    struct SensorMeasurements {
        AngleOfAttackMeasurement alpha;
        TranslationalAccelerationMeasurement fB;
        AngularVelocityMeasurement wB_BI;
        StagnationAirPressureMeasurement P0;
        StaticAirPressureMeasurement P;
        StagnationAirTemperatureMeasurement T0;
        PositionMeasurement pI_BI_gnss;
        TranslationalVelocityMeasurement vB_BI_gnss;
        HeadingMeasurement heading_BE;
    };

    struct SensorManagerInput {
        const SensorGroundTruth& ground_truth;
        double dt;
    };

    struct SensorManagerOutput {
        SensorMeasurements measurements;
    };

}
