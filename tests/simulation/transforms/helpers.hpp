#pragma once

#include <string>
#include <vector>
#include "simulation/transforms/public/data/types.hpp"

namespace transforms_test {

inline const std::vector<transforms::EulerOrder> euler_orders = {
    transforms::EulerOrder::ZYX, transforms::EulerOrder::ZXY, transforms::EulerOrder::YZX,
    transforms::EulerOrder::YXZ, transforms::EulerOrder::XZY, transforms::EulerOrder::XYZ,
    transforms::EulerOrder::ZXZ, transforms::EulerOrder::ZYZ, transforms::EulerOrder::XYX,
    transforms::EulerOrder::XZX, transforms::EulerOrder::YXY, transforms::EulerOrder::YZY
};

inline std::string euler_order_to_string(transforms::EulerOrder order) {
    switch (order) {
        case transforms::EulerOrder::ZYX: return "ZYX";
        case transforms::EulerOrder::ZXY: return "ZXY";
        case transforms::EulerOrder::YZX: return "YZX";
        case transforms::EulerOrder::YXZ: return "YXZ";
        case transforms::EulerOrder::XZY: return "XZY";
        case transforms::EulerOrder::XYZ: return "XYZ";
        case transforms::EulerOrder::ZXZ: return "ZXZ";
        case transforms::EulerOrder::ZYZ: return "ZYZ";
        case transforms::EulerOrder::XYX: return "XYX";
        case transforms::EulerOrder::XZX: return "XZX";
        case transforms::EulerOrder::YXY: return "YXY";
        case transforms::EulerOrder::YZY: return "YZY";
    }
    return "UNKNOWN";
}

}
