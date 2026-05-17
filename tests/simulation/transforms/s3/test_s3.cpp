#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <stdexcept>

#include "simulation/transforms/s3/public.hpp"
#include "simulation/transforms/s3/private.hpp"
#include "simulation/constants/public.hpp"

namespace {

    void expect_same_rotation(const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
        const Eigen::Matrix3d R1 = transforms::quat_to_rot(q1);
        const Eigen::Matrix3d R2 = transforms::quat_to_rot(q2);
        EXPECT_TRUE(R1.isApprox(R2, constants::eps_strict));
    }

    void expect_quat_near(const Eigen::Quaterniond& a, const Eigen::Quaterniond& b) {
        EXPECT_NEAR(a.w(), b.w(), constants::eps_strict);
        EXPECT_NEAR(a.x(), b.x(), constants::eps_strict);
        EXPECT_NEAR(a.y(), b.y(), constants::eps_strict);
        EXPECT_NEAR(a.z(), b.z(), constants::eps_strict);
    }

    void expect_valid_rotation_matrix(const Eigen::Matrix3d& R) {
        EXPECT_TRUE((R.transpose() * R).isApprox(Eigen::Matrix3d::Identity(), constants::eps_strict));
        EXPECT_NEAR(R.determinant(), 1.0, constants::eps_strict);
    }

    const std::vector<std::string> euler_orders = {
        "ZYX", "ZXY", "YZX", "YXZ", "XZY", "XYZ",
        "ZXZ", "ZYZ", "XYX", "XZX", "YXY", "YZY"
    };
}

TEST(S3Transforms, EulerToQuatToEulerToQuatRoundTripExtrinsic) {
    constexpr double a = 0.31;
    constexpr double b = 0.47;
    constexpr double c = -0.63;

    for (const auto& order : euler_orders) {
        SCOPED_TRACE(order);

        const auto qC1 = transforms::eul_to_quatC(a, b, c, order, "extr");
        const auto eulC = transforms::quatC_to_eul(qC1, order, "extr");
        const auto qC2 = transforms::eul_to_quatC(eulC.x(), eulC.y(), eulC.z(), order, "extr");
        expect_same_rotation(qC1, qC2);

        const auto qR1 = transforms::eul_to_quatR(a, b, c, order, "extr");
        const auto eulR = transforms::quatR_to_eul_extr(qR1, order);
        const auto qR2 = transforms::eul_to_quatR(eulR.x(), eulR.y(), eulR.z(), order, "extr");
        expect_same_rotation(qR1, qR2);
    }
}

TEST(S3Transforms, EulerToQuatToEulerToQuatRoundTripIntrinsic) {
    constexpr double a = 0.31;
    constexpr double b = 0.47;
    constexpr double c = -0.63;

    for (const auto& order : euler_orders) {
        SCOPED_TRACE(order);

        const auto qC1 = transforms::eul_to_quatC(a, b, c, order, "intr");
        const auto eulC = transforms::quatC_to_eul(qC1, order, "intr");
        const auto qC2 = transforms::eul_to_quatC(eulC.x(), eulC.y(), eulC.z(), order, "intr");
        expect_same_rotation(qC1, qC2);

        const auto qR1 = transforms::eul_to_quatR(a, b, c, order, "intr");
        const auto eulR = transforms::quatR_to_eul_intr(qR1, order);
        const auto qR2 = transforms::eul_to_quatR(eulR.x(), eulR.y(), eulR.z(), order, "intr");
        expect_same_rotation(qR1, qR2);
    }
}

TEST(S3Transforms, NormalizeAndCanonicalizeReturnsUnitQuaternionWithPositiveW) {
    Eigen::Quaterniond q(-2.0, 0.5, -1.0, 0.25);

    const auto out = transforms::normalize_and_canonicalize(q);

    EXPECT_NEAR(out.norm(), 1.0, constants::eps_strict);
    EXPECT_GE(out.w(), 0.0);
}

TEST(S3Transforms, QuatCIsConjugateOfQuatR) {
    constexpr double a = 0.2;
    constexpr double b = -0.4;
    constexpr double c = 0.7;

    const auto qR_extr = transforms::eul_to_quatR(a, b, c, "ZYX", "extr");
    const auto qC_extr = transforms::eul_to_quatC(a, b, c, "ZYX", "extr");
    expect_quat_near(qC_extr, transforms::normalize_and_canonicalize(qR_extr.conjugate()));

    const auto qR_intr = transforms::eul_to_quatR(a, b, c, "ZYX", "intr");
    const auto qC_intr = transforms::eul_to_quatC(a, b, c, "ZYX", "intr");
    expect_quat_near(qC_intr, transforms::normalize_and_canonicalize(qR_intr.conjugate()));
}

TEST(S3Transforms, QuatToRotNormalizesInput) {
    auto qR_extr = transforms::eul_to_quatR(0.3, -0.2, 0.5, "ZYX", "extr");
    qR_extr.coeffs() *= 3.0;
    expect_valid_rotation_matrix(transforms::quat_to_rot(qR_extr));

    auto qR_intr = transforms::eul_to_quatR(0.3, -0.2, 0.5, "ZYX", "intr");
    qR_intr.coeffs() *= 3.0;
    expect_valid_rotation_matrix(transforms::quat_to_rot(qR_intr));

    auto qC_extr = transforms::eul_to_quatC(0.3, -0.2, 0.5, "ZYX", "extr");
    qC_extr.coeffs() *= 3.0;
    expect_valid_rotation_matrix(transforms::quat_to_rot(qC_extr));

    auto qC_intr = transforms::eul_to_quatC(0.3, -0.2, 0.5, "ZYX", "intr");
    qC_intr.coeffs() *= 3.0;
    expect_valid_rotation_matrix(transforms::quat_to_rot(qC_intr));
}

TEST(S3Transforms, IntrinsicAndExtrinsicWrappersDispatch) {
    constexpr double a = 0.1;
    constexpr double b = 0.2;
    constexpr double c = -0.3;

    expect_quat_near(transforms::eul_to_quatR(a, b, c, "XYZ", "extr"), transforms::eul_to_quatR_extr(a, b, c, "XYZ"));
    expect_quat_near(transforms::eul_to_quatR(a, b, c, "XYZ", "intr"), transforms::eul_to_quatR_intr(a, b, c, "XYZ"));
    expect_quat_near(transforms::eul_to_quatC(a, b, c, "XYZ", "extr"), transforms::eul_to_quatC_extr(a, b, c, "XYZ"));
    expect_quat_near(transforms::eul_to_quatC(a, b, c, "XYZ", "intr"), transforms::eul_to_quatC_intr(a, b, c, "XYZ"));
}

TEST(S3Transforms, RejectsInvalidTypeArgument) {
    EXPECT_THROW(transforms::eul_to_quatR(0.0, 0.0, 0.0, "ZYX", "bad"), std::invalid_argument);
    EXPECT_THROW(transforms::eul_to_quatC(0.0, 0.0, 0.0, "ZYX", "bad"), std::invalid_argument);
    EXPECT_THROW(transforms::quatC_to_eul(Eigen::Quaterniond::Identity(), "ZYX", "bad"), std::invalid_argument);
}

TEST(S3Transforms, RejectsInvalidEulerOrderArgument) {
    EXPECT_THROW(transforms::eul_to_quatR(0.0, 0.0, 0.0, "BAD", "extr"), std::invalid_argument);
    EXPECT_THROW(transforms::eul_to_quatR(0.0, 0.0, 0.0, "BAD", "intr"), std::invalid_argument);
    EXPECT_THROW(transforms::eul_to_quatC(0.0, 0.0, 0.0, "BAD", "extr"), std::invalid_argument);
    EXPECT_THROW(transforms::eul_to_quatC(0.0, 0.0, 0.0, "BAD", "intr"), std::invalid_argument);
}
