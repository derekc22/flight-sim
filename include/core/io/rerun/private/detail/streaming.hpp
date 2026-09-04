#pragma once
#include "core/io/rerun/public/manager.hpp"
#include "simulation/dynamics/public/data/types.hpp"

#include <Eigen/Dense>
#include <opencv2/core/mat.hpp>
#include <rerun.hpp>
#include <string>
#include <vector>

namespace io
{

	extern const std::vector<std::string> xyz_labels;
	extern const std::vector<std::string> eul_labels;
	extern const std::vector<std::string> angular_rate_labels;
	extern const std::vector<std::string> velocity_labels;
	extern const std::vector<std::string> surface_labels;
	extern const std::vector<std::string> propulsor_labels;

	void stream_scalar(rerun::RecordingStream& rec, const std::string& path, double value);
	void stream_vector(rerun::RecordingStream& rec,
		const std::string& prefix,
		const Eigen::Ref<const Eigen::VectorXd>& data,
		const std::vector<std::string>& labels);

	rerun::Vector3D scaled_vector(const Eigen::Vector3d& data, double scale);

	void stream_vehicle_transform(rerun::RecordingStream& rec, const dynamics::RigidBodyState& Xt);
	void stream_estimated_vehicle_transform(rerun::RecordingStream& rec, const dynamics::RigidBodyState& Zt);

	void stream_body_arrow(rerun::RecordingStream& rec,
		const std::string& path,
		const Eigen::Vector3d& data,
		double scale,
		const rerun::Color& color,
		const std::string& label);

	void stream_vehicle_model(rerun::RecordingStream& rec,
		const Eigen::Quaterniond& q_model_to_body,
		const std::string& path);

	void clip_trajectory(std::vector<rerun::Vec3D>& trajectory, std::size_t max_size);

	void stream_vehicle_trajectory(rerun::RecordingStream& rec, const std::vector<rerun::Vec3D>& trajectory);
	void stream_estimated_vehicle_trajectory(rerun::RecordingStream& rec,
		const std::vector<rerun::Vec3D>& estimated_trajectory);

	void stream_flightgear_image(rerun::RecordingStream& rec, const cv::Mat& image);
} // namespace io
