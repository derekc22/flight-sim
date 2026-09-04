#pragma once
#include "core/io/data/public/data/types.hpp"

#include <opencv2/core/mat.hpp>

namespace io
{

	struct RerunManagerInput {
		int t;
		const DataContext& data_context;
	};

	struct RerunContext {
		int t;
		DataContext data_context;
		cv::Mat image;
	};

} // namespace io
