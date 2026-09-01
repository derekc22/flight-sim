#pragma once
#include <opencv2/core/mat.hpp>
#include "core/io/data/public/data/types.hpp"

namespace io {

    struct RerunContext {
        int t;
        DataContext data_context;
        cv::Mat image;
    };

}
