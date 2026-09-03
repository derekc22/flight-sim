#include <optional>
#include "simulation/atmospheric/public/detail/compressible.hpp"
#include "simulation/avionics/public/manager.hpp"

namespace avionics {

	AvionicsManagerOutput AvionicsManager::step(
	    const AvionicsManagerInput& input)
	{
		const sensors::SensorMeasurements& sensor_meas = input.sensor_meas;
		const std::optional<sensors::SensorMeasurements>& sensor_hist = input.sensor_hist;
		const sensors::SensorGroundTruth& sensor_gt = input.sensor_gt;
		const AvionicsGroundTruth& avionics_gt = input.avionics_gt;
		double dt = input.dt;

		MachNumberMeasurement mach_meas{atmospheric::compute_mach(sensor_meas.P0, sensor_meas.P)};
		StaticAirTemperatureMeasurement T_meas{atmospheric::T_from_T0(sensor_meas.T0, mach_meas)};

		AvionicsMeasurements avionics_meas{.pI_BI_ins = hist
		        ? avionics.INS.compute(hist->pI_BI_ins, hist->vB_BI_ins, sensor_meas.fB, avionics_gt.gB, hist->qIB, dt)
		        : sensors::PositionMeasurement{sensor_gt.pI_BI},
		    .vB_BI_ins = hist
		        ? avionics.INS.compute(hist->vB_BI_ins, sensor_meas.fB, avionics_gt.gB, sensor_meas.wB_BI, dt)
		        : sensors::TranslationalVelocityMeasurement{sensor_gt.vB_BI},
		    .T = T_meas,
		    .Mach = mach_meas,
		    .qIB = hist ? avionics.AHRS.compute(hist->qIB, sensor_meas.wB_BI, sensor_meas.fB, dt)
		                : OrientationMeasurement{avionics_gt.qIB},
		    .Vinf = avionics.ADC.compute(mach_meas, T_meas),
		    .pressure_alt_BE = avionics.ADC.compute(sensor_meas.P),
		    .alt_BE_dot = sensor_hist ? avionics.ADC.compute(sensor_meas.P, sensor_hist->P, T_meas, dt)
		                              : VerticalSpeedMeasurement{avionics_gt.alt_BE_dot},
		    .rho = avionics.ADC.compute(sensor_meas.P, T_meas)};

		hist = avionics_meas;

		return {.avionics_meas = avionics_meas};
	}
} // namespace avionics
