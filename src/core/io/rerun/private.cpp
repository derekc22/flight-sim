#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <rerun.hpp>
#include "core/io/rerun/private.hpp"
#include "simulation/dynamics/public.hpp"
#include <opencv2/opencv.hpp>

namespace io {

    const std::vector<std::string> xyz_labels{"x", "y", "z"};
    const std::vector<std::string> eul_labels{"psi", "theta", "phi"};
    const std::vector<std::string> angular_rate_labels{"p", "q", "r"};
    const std::vector<std::string> velocity_labels{"vx", "vy", "vz"};
    const std::vector<std::string> surface_labels{"elevator", "aileron", "rudder", "flap", "spoiler"};
    const std::vector<std::string> propulsor_labels{"front", "left", "right"};

    void stream_scalar(rerun::RecordingStream& rec, const std::string& path, double value) {
        rec.log(path, rerun::Scalars(value));
    }

    void stream_vector(rerun::RecordingStream& rec, const std::string& prefix, const Eigen::Ref<const Eigen::VectorXd>& data, const std::vector<std::string>& labels) {
        Eigen::Index n = std::min<Eigen::Index>(data.size(), static_cast<Eigen::Index>(labels.size()));
        for (Eigen::Index i = 0; i < n; ++i) {
            stream_scalar(rec, prefix + "/" + labels[static_cast<std::size_t>(i)], data(i));
        }
    }

    rerun::Vector3D scaled_vector(const Eigen::Vector3d& data, double scale) {
        return rerun::Vector3D(
            static_cast<float>(scale * data.x()), 
            static_cast<float>(scale * data.y()), 
            static_cast<float>(scale * data.z())
        );
    }

    void stream_vehicle_transform(rerun::RecordingStream& rec, const dynamics::RigidBodyState& Xt) {
        const Eigen::Quaterniond q_conj = Xt.q.data.conjugate();  // Rerun expects qBI, not qIB
        rec.log(
            "world/vehicle",
            rerun::Transform3D::from_translation_rotation(
                rerun::components::Translation3D(
                    static_cast<float>(Xt.p.data.x()), 
                    static_cast<float>(Xt.p.data.y()), 
                    static_cast<float>(Xt.p.data.z())
                ),
                rerun::datatypes::Quaternion::from_xyzw(
                    static_cast<float>(q_conj.x()), 
                    static_cast<float>(q_conj.y()), 
                    static_cast<float>(q_conj.z()), 
                    static_cast<float>(q_conj.w())
                )
            ),
            rerun::TransformAxes3D(5.0)
        );
    }

    void stream_estimated_vehicle_transform(rerun::RecordingStream& rec, const dynamics::RigidBodyState& Zt) {
        const Eigen::Quaterniond q_conj = Zt.q.data.conjugate();  // Rerun expects qBI, not qIB
        rec.log(
            "world/estimated_vehicle",
            rerun::Transform3D::from_translation_rotation(
                rerun::components::Translation3D(
                    static_cast<float>(Zt.p.data.x()), 
                    static_cast<float>(Zt.p.data.y()), 
                    static_cast<float>(Zt.p.data.z())
                ),
                rerun::datatypes::Quaternion::from_xyzw(
                    static_cast<float>(q_conj.x()), 
                    static_cast<float>(q_conj.y()), 
                    static_cast<float>(q_conj.z()), 
                    static_cast<float>(q_conj.w())
                )
            ),
            rerun::TransformAxes3D(5.0)
        );
    }

    void stream_body_arrow(rerun::RecordingStream& rec, const std::string& path, const Eigen::Vector3d& data, double scale, const rerun::Color& color, const std::string& label) {
        rec.log(
            path,
            rerun::Arrows3D::from_vectors({scaled_vector(data, scale)})
                .with_origins({rerun::Position3D(0.0f, 0.0f, 0.0f)})
                .with_colors({color})
                .with_radii({rerun::Radius::ui_points(2.0f)})
                // .with_labels({label})
        );
    }

    void stream_vehicle_model(rerun::RecordingStream& rec, const Eigen::Quaterniond& q_model_to_body, const std::string& path) {
        rec.log_static(
            path,
            rerun::Transform3D::from_translation_rotation(
                rerun::components::Translation3D(0.0f, 0.0f, 0.0f),
                rerun::datatypes::Quaternion::from_xyzw(
                    static_cast<float>(q_model_to_body.x()),
                    static_cast<float>(q_model_to_body.y()),
                    static_cast<float>(q_model_to_body.z()),
                    static_cast<float>(q_model_to_body.w())
                )
            )
        );

        rec.log_static(
            path + std::string("/model"),
            rerun::Asset3D::from_file_path("assets/cessna172.glb").value
        );
    }

    void clip_trajectory(std::vector<rerun::Vec3D>& trajectory, std::size_t max_size) {
        if (trajectory.size() > max_size) {
            trajectory.erase(trajectory.begin());
        }
    }

    void stream_vehicle_trajectory(rerun::RecordingStream& rec, const std::vector<rerun::Vec3D>& trajectory) {
        if (trajectory.size() < 2) { return; }

        rerun::LineStrip3D strip(trajectory);

        rec.log(
            "world/trajectory",
            rerun::LineStrips3D(strip)
                .with_radii(rerun::Radius::ui_points(0.5f))
                .with_colors(rerun::Color(255, 255, 0))
        );
    }

    void stream_estimated_vehicle_trajectory(rerun::RecordingStream& rec, const std::vector<rerun::Vec3D>& estimated_trajectory) {
        if (estimated_trajectory.size() < 2) { return; }

        rerun::LineStrip3D strip(estimated_trajectory);

        rec.log(
            "world/estimated_trajectory",
            rerun::LineStrips3D(strip)
                .with_radii(rerun::Radius::ui_points(0.5f))
                .with_colors(rerun::Color(255, 255, 0))
        );
    }

    void stream_flightgear_image(rerun::RecordingStream& rec, const cv::Mat& image) {
        if (image.empty()) {
            throw std::runtime_error("stream_flightgear_image: empty image");
        }

        if (image.type() != CV_8UC3) {
            throw std::runtime_error("stream_flightgear_image: expected CV_8UC3 BGR image");
        }

        const cv::Mat img = image.isContinuous() ? image : image.clone();

        rec.log(
            "flightgear/camera",
            rerun::Image(
                rerun::borrow(img.data, img.total() * img.elemSize()),
                rerun::WidthHeight(
                    static_cast<uint32_t>(img.cols),
                    static_cast<uint32_t>(img.rows)
                ),
                rerun::ColorModel::BGR
            )
        );
    }

}
