#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <stdexcept>

#include "simulation/transforms/so3/public.hpp"
#include "simulation/transforms/so3/private.hpp"
#include "simulation/transforms/s3/public.hpp"
#include "simulation/constants/public.hpp"
#include "simulation/transforms/helpers.hpp"

static void expect_matrix_near(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
    EXPECT_TRUE(A.isApprox(B, constants::eps_strict));
}

static void expect_orthonormal_with_unit_determinant(const Eigen::Matrix3d& R) {
    EXPECT_TRUE((R.transpose() * R).isApprox(constants::I_T<double, 3>, constants::eps_strict));
    EXPECT_NEAR(R.determinant(), 1.0, constants::eps_strict);
}

TEST(transforms_so3, EulerToRotToEulerToRotRoundTripExtrinsic) {
    constexpr double a = 0.31;
    constexpr double b = 0.47;
    constexpr double c = -0.63;

    for (const auto& order : transforms_test::euler_orders) {
        SCOPED_TRACE(transforms_test::euler_order_to_string(order));

        const auto C1 = transforms::eul_to_C(a, b, c, order, transforms::RotationType::Extrinsic);
        const auto eulC = transforms::C_to_eul(C1, order, transforms::RotationType::Extrinsic);
        const auto C2 = transforms::eul_to_C(eulC.x(), eulC.y(), eulC.z(), order, transforms::RotationType::Extrinsic);
        expect_matrix_near(C1, C2);

        const auto R1 = transforms::eul_to_R(a, b, c, order);
        const auto eulR = transforms::R_to_eul_extr(R1, order);
        const auto R2 = transforms::eul_to_R(eulR.x(), eulR.y(), eulR.z(), order);
        expect_matrix_near(R1, R2);
    }
}

TEST(transforms_so3, EulerToRotToEulerToRotRoundTripIntrinsic) {
    constexpr double a = 0.31;
    constexpr double b = 0.47;
    constexpr double c = -0.63;

    for (const auto& order : transforms_test::euler_orders) {
        SCOPED_TRACE(transforms_test::euler_order_to_string(order));

        const auto C1 = transforms::eul_to_C(a, b, c, order, transforms::RotationType::Intrinsic);
        const auto eulC = transforms::C_to_eul(C1, order, transforms::RotationType::Intrinsic);
        const auto C2 = transforms::eul_to_C(eulC.x(), eulC.y(), eulC.z(), order, transforms::RotationType::Intrinsic);
        expect_matrix_near(C1, C2);

        const auto R1 = transforms::eul_to_R_intr(a, b, c, order);
        const auto eulR = transforms::R_to_eul_intr(R1, order);
        const auto R2 = transforms::eul_to_R_intr(eulR.x(), eulR.y(), eulR.z(), order);
        expect_matrix_near(R1, R2);
    }
}

TEST(transforms_so3, CIsTransposeOfR) {
    constexpr double a = 0.2;
    constexpr double b = -0.4;
    constexpr double c = 0.7;

    const auto R_extr = transforms::eul_to_R_extr(a, b, c, transforms::EulerOrder::ZYX);
    const auto C_extr = transforms::eul_to_C(a, b, c, transforms::EulerOrder::ZYX, transforms::RotationType::Extrinsic);
    expect_matrix_near(C_extr, R_extr.transpose());

    const auto R_intr = transforms::eul_to_R_intr(a, b, c, transforms::EulerOrder::ZYX);
    const auto C_intr = transforms::eul_to_C(a, b, c, transforms::EulerOrder::ZYX, transforms::RotationType::Intrinsic);
    expect_matrix_near(C_intr, R_intr.transpose());
}

TEST(transforms_so3, ChainRotPostAndPreComposeInExpectedOrder) {
    const auto expect_chain_order = [](const Eigen::Matrix3d& rot1, const Eigen::Matrix3d& rot2) {
        const std::vector<Eigen::Matrix3d> rot_list = {rot1, rot2};
        expect_matrix_near(transforms::chain_rot_post(rot_list), rot1 * rot2);
        expect_matrix_near(transforms::chain_rot_pre(rot_list), rot2 * rot1);
    };

    expect_chain_order(transforms::eul_to_R(0.2, -0.3, 0.4, transforms::EulerOrder::ZYX), transforms::eul_to_R(-0.1, 0.5, 0.2, transforms::EulerOrder::XYZ));
    expect_chain_order(transforms::eul_to_R_intr(0.2, -0.3, 0.4, transforms::EulerOrder::ZYX), transforms::eul_to_R_intr(-0.1, 0.5, 0.2, transforms::EulerOrder::XYZ));
    expect_chain_order(transforms::eul_to_C(0.2, -0.3, 0.4, transforms::EulerOrder::ZYX, transforms::RotationType::Extrinsic), transforms::eul_to_C(-0.1, 0.5, 0.2, transforms::EulerOrder::XYZ, transforms::RotationType::Extrinsic));
    expect_chain_order(transforms::eul_to_C(0.2, -0.3, 0.4, transforms::EulerOrder::ZYX, transforms::RotationType::Intrinsic), transforms::eul_to_C(-0.1, 0.5, 0.2, transforms::EulerOrder::XYZ, transforms::RotationType::Intrinsic));
}

TEST(transforms_so3, RotToQuatToRotRoundTrip) {
    const auto R = transforms::eul_to_R(0.3, -0.2, 0.5, transforms::EulerOrder::ZYX);
    const auto q = transforms::rot_to_quat(R);

    EXPECT_NEAR(q.norm(), 1.0, constants::eps_strict);
    expect_matrix_near(transforms::quat_to_rot(q), R);
}

TEST(transforms_so3, EulerToRAndEulerToCReturnOrthonormalMatricesWithUnitDeterminant) {
    expect_orthonormal_with_unit_determinant(transforms::eul_to_R(0.3, -0.2, 0.5, transforms::EulerOrder::ZYX));
    expect_orthonormal_with_unit_determinant(transforms::eul_to_R_intr(0.3, -0.2, 0.5, transforms::EulerOrder::ZYX));
    expect_orthonormal_with_unit_determinant(transforms::eul_to_C(0.3, -0.2, 0.5, transforms::EulerOrder::ZYX, transforms::RotationType::Extrinsic));
    expect_orthonormal_with_unit_determinant(transforms::eul_to_C(0.3, -0.2, 0.5, transforms::EulerOrder::ZYX, transforms::RotationType::Intrinsic));
}

TEST(transforms_so3, IntrinsicAndExtrinsicWrappersDispatch) {
    constexpr double a = 0.1;
    constexpr double b = 0.2;
    constexpr double c = -0.3;

    expect_matrix_near(transforms::eul_to_R(a, b, c, transforms::EulerOrder::XYZ), transforms::eul_to_R_extr(a, b, c, transforms::EulerOrder::XYZ));
    expect_matrix_near(transforms::eul_to_C(a, b, c, transforms::EulerOrder::XYZ, transforms::RotationType::Extrinsic), transforms::eul_to_C_extr(a, b, c, transforms::EulerOrder::XYZ));
    expect_matrix_near(transforms::eul_to_C(a, b, c, transforms::EulerOrder::XYZ, transforms::RotationType::Intrinsic), transforms::eul_to_C_intr(a, b, c, transforms::EulerOrder::XYZ));
}

TEST(transforms_so3, RejectsInvalidTypeArgument) {
    EXPECT_THROW(transforms::eul_to_C(0.0, 0.0, 0.0, transforms::EulerOrder::ZYX, static_cast<transforms::RotationType>(-1)), std::invalid_argument);
    EXPECT_THROW(transforms::C_to_eul(constants::I_T<double, 3>, transforms::EulerOrder::ZYX, static_cast<transforms::RotationType>(-1)), std::invalid_argument);
}

TEST(transforms_so3, RejectsInvalidEulerOrderArgument) {
    EXPECT_THROW(transforms::eul_to_R(0.0, 0.0, 0.0, static_cast<transforms::EulerOrder>(-1)), std::invalid_argument);
    EXPECT_THROW(transforms::eul_to_R_intr(0.0, 0.0, 0.0, static_cast<transforms::EulerOrder>(-1)), std::invalid_argument);
    EXPECT_THROW(transforms::eul_to_C(0.0, 0.0, 0.0, static_cast<transforms::EulerOrder>(-1), transforms::RotationType::Extrinsic), std::invalid_argument);
    EXPECT_THROW(transforms::eul_to_C(0.0, 0.0, 0.0, static_cast<transforms::EulerOrder>(-1), transforms::RotationType::Intrinsic), std::invalid_argument);
}
