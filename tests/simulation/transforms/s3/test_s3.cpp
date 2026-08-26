#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <stdexcept>

#include "simulation/transforms/s3/public.hpp"
#include "simulation/transforms/s3/private.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/transforms/helpers.hpp"

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
        EXPECT_TRUE((R.transpose() * R).isApprox(constants::I_T<double, 3>, constants::eps_strict));
        EXPECT_NEAR(R.determinant(), 1.0, constants::eps_strict);
    }
}

TEST(transforms_s3, EulerToQuatToEulerToQuatRoundTripExtrinsic) {
    constexpr double a = 0.31;
    constexpr double b = 0.47;
    constexpr double c = -0.63;

    for (const auto& order : transforms_test::euler_orders) {
        SCOPED_TRACE(transforms_test::euler_order_to_string(order));

        const auto qC1 = transforms::eul_to_quatC(a, b, c, order, transforms::RotationType::Extrinsic);
        const auto eulC = transforms::quatC_to_eul(qC1, order, transforms::RotationType::Extrinsic);
        const auto qC2 = transforms::eul_to_quatC(eulC.x(), eulC.y(), eulC.z(), order, transforms::RotationType::Extrinsic);
        expect_same_rotation(qC1, qC2);

        const auto qR1 = transforms::eul_to_quatR(a, b, c, order);
        const auto eulR = transforms::quatR_to_eul_extr(qR1, order);
        const auto qR2 = transforms::eul_to_quatR(eulR.x(), eulR.y(), eulR.z(), order);
        expect_same_rotation(qR1, qR2);
    }
}

TEST(transforms_s3, EulerToQuatToEulerToQuatRoundTripIntrinsic) {
    constexpr double a = 0.31;
    constexpr double b = 0.47;
    constexpr double c = -0.63;

    for (const auto& order : transforms_test::euler_orders) {
        SCOPED_TRACE(transforms_test::euler_order_to_string(order));

        const auto qC1 = transforms::eul_to_quatC(a, b, c, order, transforms::RotationType::Intrinsic);
        const auto eulC = transforms::quatC_to_eul(qC1, order, transforms::RotationType::Intrinsic);
        const auto qC2 = transforms::eul_to_quatC(eulC.x(), eulC.y(), eulC.z(), order, transforms::RotationType::Intrinsic);
        expect_same_rotation(qC1, qC2);

        const auto qR1 = transforms::eul_to_quatR_intr(a, b, c, order);
        const auto eulR = transforms::quatR_to_eul_intr(qR1, order);
        const auto qR2 = transforms::eul_to_quatR_intr(eulR.x(), eulR.y(), eulR.z(), order);
        expect_same_rotation(qR1, qR2);
    }
}

TEST(transforms_s3, NormalizeAndCanonicalizeReturnsUnitQuaternionWithPositiveW) {
    Eigen::Quaterniond q(-2.0, 0.5, -1.0, 0.25);

    const auto out = transforms::normalize_and_canonicalize(q);

    EXPECT_NEAR(out.norm(), 1.0, constants::eps_strict);
    EXPECT_GE(out.w(), 0.0);
}

TEST(transforms_s3, QuatCIsConjugateOfQuatR) {
    constexpr double a = 0.2;
    constexpr double b = -0.4;
    constexpr double c = 0.7;

    const auto qR_extr = transforms::eul_to_quatR_extr(a, b, c, transforms::EulerOrder::ZYX);
    const auto qC_extr = transforms::eul_to_quatC(a, b, c, transforms::EulerOrder::ZYX, transforms::RotationType::Extrinsic);
    expect_quat_near(qC_extr, transforms::normalize_and_canonicalize(qR_extr.conjugate()));

    const auto qR_intr = transforms::eul_to_quatR_intr(a, b, c, transforms::EulerOrder::ZYX);
    const auto qC_intr = transforms::eul_to_quatC(a, b, c, transforms::EulerOrder::ZYX, transforms::RotationType::Intrinsic);
    expect_quat_near(qC_intr, transforms::normalize_and_canonicalize(qR_intr.conjugate()));
}

TEST(transforms_s3, ChainQuatPostAndPreComposeInExpectedOrder) {
    const auto expect_chain_order = [](const Eigen::Quaterniond& q1, const Eigen::Quaterniond& q2) {
        const std::vector<Eigen::Quaterniond> q_list = {q1, q2};
        expect_quat_near(transforms::chain_quat_post(q_list), transforms::normalize_and_canonicalize(q1 * q2));
        expect_quat_near(transforms::chain_quat_pre(q_list), transforms::normalize_and_canonicalize(q2 * q1));
    };

    expect_chain_order(transforms::eul_to_quatR(0.2, -0.3, 0.4, transforms::EulerOrder::ZYX), transforms::eul_to_quatR(-0.1, 0.5, 0.2, transforms::EulerOrder::XYZ));
    expect_chain_order(transforms::eul_to_quatR_intr(0.2, -0.3, 0.4, transforms::EulerOrder::ZYX), transforms::eul_to_quatR_intr(-0.1, 0.5, 0.2, transforms::EulerOrder::XYZ));
    expect_chain_order(transforms::eul_to_quatC(0.2, -0.3, 0.4, transforms::EulerOrder::ZYX, transforms::RotationType::Extrinsic), transforms::eul_to_quatC(-0.1, 0.5, 0.2, transforms::EulerOrder::XYZ, transforms::RotationType::Extrinsic));
    expect_chain_order(transforms::eul_to_quatC(0.2, -0.3, 0.4, transforms::EulerOrder::ZYX, transforms::RotationType::Intrinsic), transforms::eul_to_quatC(-0.1, 0.5, 0.2, transforms::EulerOrder::XYZ, transforms::RotationType::Intrinsic));
}

TEST(transforms_s3, QuatToRotNormalizesInput) {
    auto qR_extr = transforms::eul_to_quatR(0.3, -0.2, 0.5, transforms::EulerOrder::ZYX);
    qR_extr.coeffs() *= 3.0;
    expect_valid_rotation_matrix(transforms::quat_to_rot(qR_extr));

    auto qR_intr = transforms::eul_to_quatR_intr(0.3, -0.2, 0.5, transforms::EulerOrder::ZYX);
    qR_intr.coeffs() *= 3.0;
    expect_valid_rotation_matrix(transforms::quat_to_rot(qR_intr));

    auto qC_extr = transforms::eul_to_quatC(0.3, -0.2, 0.5, transforms::EulerOrder::ZYX, transforms::RotationType::Extrinsic);
    qC_extr.coeffs() *= 3.0;
    expect_valid_rotation_matrix(transforms::quat_to_rot(qC_extr));

    auto qC_intr = transforms::eul_to_quatC(0.3, -0.2, 0.5, transforms::EulerOrder::ZYX, transforms::RotationType::Intrinsic);
    qC_intr.coeffs() *= 3.0;
    expect_valid_rotation_matrix(transforms::quat_to_rot(qC_intr));
}

TEST(transforms_s3, IntrinsicAndExtrinsicWrappersDispatch) {
    constexpr double a = 0.1;
    constexpr double b = 0.2;
    constexpr double c = -0.3;

    expect_quat_near(transforms::eul_to_quatR(a, b, c, transforms::EulerOrder::XYZ), transforms::eul_to_quatR_extr(a, b, c, transforms::EulerOrder::XYZ));
    expect_quat_near(transforms::eul_to_quatC(a, b, c, transforms::EulerOrder::XYZ, transforms::RotationType::Extrinsic), transforms::eul_to_quatC_extr(a, b, c, transforms::EulerOrder::XYZ));
    expect_quat_near(transforms::eul_to_quatC(a, b, c, transforms::EulerOrder::XYZ, transforms::RotationType::Intrinsic), transforms::eul_to_quatC_intr(a, b, c, transforms::EulerOrder::XYZ));
}

TEST(transforms_s3, RejectsInvalidTypeArgument) {
    EXPECT_THROW(transforms::eul_to_quatC(0.0, 0.0, 0.0, transforms::EulerOrder::ZYX, static_cast<transforms::RotationType>(-1)), std::invalid_argument);
    EXPECT_THROW(transforms::quatC_to_eul(Eigen::Quaterniond::Identity(), transforms::EulerOrder::ZYX, static_cast<transforms::RotationType>(-1)), std::invalid_argument);
}

TEST(transforms_s3, RejectsInvalidEulerOrderArgument) {
    EXPECT_THROW(transforms::eul_to_quatR(0.0, 0.0, 0.0, static_cast<transforms::EulerOrder>(-1)), std::invalid_argument);
    EXPECT_THROW(transforms::eul_to_quatR_intr(0.0, 0.0, 0.0, static_cast<transforms::EulerOrder>(-1)), std::invalid_argument);
    EXPECT_THROW(transforms::eul_to_quatC(0.0, 0.0, 0.0, static_cast<transforms::EulerOrder>(-1), transforms::RotationType::Extrinsic), std::invalid_argument);
    EXPECT_THROW(transforms::eul_to_quatC(0.0, 0.0, 0.0, static_cast<transforms::EulerOrder>(-1), transforms::RotationType::Intrinsic), std::invalid_argument);
}
