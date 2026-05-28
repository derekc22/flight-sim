#pragma once

namespace json {

    template <typename Estimator, typename EstimatorClass, typename EstimatorParameters, typename EstimatorClassInput>
    EstimatorClass make_stateful_estimator(const EstimatorParameters& params) {
        return [estimator = Estimator{ params }](const EstimatorClassInput& input) mutable {
            return estimator.step(input);
        };
    }
}
