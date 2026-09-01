#include <stdexcept>
#include "simulation/guidance/public/manager.hpp"

namespace guidance {

    GuidanceManagerOutput GuidanceManager::step(const GuidanceManagerInput& input) {
        if (regulation.has_value()) {
            return { .setpoint = regulation.value().step() };
        }
        if (tracking.has_value()) {
            return { .setpoint = tracking.value().step() };
        }
        if (interpolated.has_value()) {
            return { .setpoint = interpolated.value().step(input.kf) };
        }
        throw std::runtime_error("control::step invalid trajectory type");
    }

    GuidanceSetpoint GuidanceManager::step(int kf) {
        return step({ .kf = kf }).setpoint;
    }

}
