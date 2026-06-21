#pragma once
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <rerun.hpp>
#include "core/io/rerun/public.hpp"
#include "simulation/dynamics/public.hpp"

namespace io {

    extern const std::vector<std::string> xyz_labels;
    extern const std::vector<std::string> eul_labels;
    extern const std::vector<std::string> angular_rate_labels;
    extern const std::vector<std::string> velocity_labels;
    extern const std::vector<std::string> surface_labels;
    extern const std::vector<std::string> propulsor_labels;

    void log_scalar(rerun::RecordingStream& rec, const std::string& path, double value);
    void log_vector(rerun::RecordingStream& rec, const std::string& prefix, const Eigen::Ref<const Eigen::VectorXd>& data, const std::vector<std::string>& labels);
    rerun::Vector3D scaled_vector(const Eigen::Vector3d& data, double scale);
    void log_vehicle_transform(rerun::RecordingStream& rec, const dynamics::RigidBodyState& Xt);
    void log_body_arrow(rerun::RecordingStream& rec, const std::string& path, const Eigen::Vector3d& data, double scale, const rerun::Color& color, const std::string& label);
    void log_vehicle_model(rerun::RecordingStream& rec);
}
