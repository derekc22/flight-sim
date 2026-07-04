#pragma once

namespace json {

    template <typename Estimator, typename EstimatorClass, typename EstimatorParameters, typename EstimatorClassInput>
    EstimatorClass make_stateful_estimator(const EstimatorParameters& params) {
        // Creates and returns a stateful lambda that owns a Estimator initialized with params
        // Each call forwards the EstimatorClassInput to the stored estimator's step(input) method and returns the result
        // mutable is required because lambda objects treat captured values as const by default, but step(input) may modify the stored estimator
        return [estimator = Estimator{ params }](const EstimatorClassInput& input, double dt) mutable {
            return estimator.step(input, dt);
        };
    }
}
