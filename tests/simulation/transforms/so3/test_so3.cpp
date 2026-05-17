#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <string>
#include <vector>
#include <stdexcept>

#include "simulation/transforms/so3/public.hpp"
#include "simulation/transforms/so3/private.hpp"
#include "simulation/transforms/s3/public.hpp"
#include "simulation/constants/public.hpp"

static void expect_matrix_near(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
    EXPECT_TRUE(A.isApprox(B, constants::eps_strict));
}

static void expect_orthonormal_with_unit_determinant(const Eigen::Matrix3d& R) {
    EXPECT_TRUE((R.transpose() * R).isApprox(Eigen::Matrix3d::Identity(), constants::eps_strict));
    EXPECT_NEAR(R.determinant(), 1.0, constants::eps_strict);
}

static const std::vector<std::string> euler_orders = {
    "ZYX", "ZXY", "YZX", "YXZ", "XZY", "XYZ",
    "ZXZ", "ZYZ", "XYX", "XZX", "YXY", "YZY"
};

TEST(SO3Transforms, EulerToRotToEulerToRotRoundTripExtrinsic) {
    constexpr double a = 0.31;
    constexpr double b = 0.47;
    constexpr double c = -0.63;

    for (const auto& order : euler_orders) {
        SCOPED_TRACE(order);

        const auto C1 = transforms::eul_to_C(a, b, c, order, "extr");
        const auto eulC = transforms::C_to_eul(C1, order, "extr");
        const auto C2 = transforms::eul_to_C(eulC.x(), eulC.y(), eulC.z(), order, "extr");
        expect_matrix_near(C1, C2);

        const auto R1 = transforms::eul_to_R(a, b, c, order);
        const auto eulR = transforms::R_to_eul_extr(R1, order);
        const auto R2 = transforms::eul_to_R(eulR.x(), eulR.y(), eulR.z(), order);
        expect_matrix_near(R1, R2);
    }
}

TEST(SO3Transforms, EulerToRotToEulerToRotRoundTripIntrinsic) {
    constexpr double a = 0.31;
    constexpr double b = 0.47;
    constexpr double c = -0.63;

    for (const auto& order : euler_orders) {
        SCOPED_TRACE(order);

        const auto C1 = transforms::eul_to_C(a, b, c, order, "intr");
        const auto eulC = transforms::C_to_eul(C1, order, "intr");
        const auto C2 = transforms::eul_to_C(eulC.x(), eulC.y(), eulC.z(), order, "intr");
        expect_matrix_near(C1, C2);

        const auto R1 = transforms::eul_to_R_intr(a, b, c, order);
        const auto eulR = transforms::R_to_eul_intr(R1, order);
        const auto R2 = transforms::eul_to_R_intr(eulR.x(), eulR.y(), eulR.z(), order);
        expect_matrix_near(R1, R2);
    }
}

TEST(SO3Transforms, CIsTransposeOfR) {
    constexpr double a = 0.2;
    constexpr double b = -0.4;
    constexpr double c = 0.7;

    const auto R_extr = transforms::eul_to_R_extr(a, b, c, "ZYX");
    const auto C_extr = transforms::eul_to_C(a, b, c, "ZYX", "extr");
    expect_matrix_near(C_extr, R_extr.transpose());

    const auto R_intr = transforms::eul_to_R_intr(a, b, c, "ZYX");
    const auto C_intr = transforms::eul_to_C(a, b, c, "ZYX", "intr");
    expect_matrix_near(C_intr, R_intr.transpose());
}

TEST(SO3Transforms, RotToQuatToRotRoundTrip) {
    const auto R = transforms::eul_to_R(0.3, -0.2, 0.5, "ZYX");
    const auto q = transforms::rot_to_quat(R);

    EXPECT_NEAR(q.norm(), 1.0, constants::eps_strict);
    expect_matrix_near(transforms::quat_to_rot(q), R);
}

TEST(SO3Transforms, EulerToRAndEulerToCReturnOrthonormalMatricesWithUnitDeterminant) {
    expect_orthonormal_with_unit_determinant(transforms::eul_to_R(0.3, -0.2, 0.5, "ZYX"));
    expect_orthonormal_with_unit_determinant(transforms::eul_to_C(0.3, -0.2, 0.5, "ZYX", "extr"));
    expect_orthonormal_with_unit_determinant(transforms::eul_to_C(0.3, -0.2, 0.5, "ZYX", "intr"));
}

TEST(SO3Transforms, IntrinsicAndExtrinsicWrappersDispatch) {
    constexpr double a = 0.1;
    constexpr double b = 0.2;
    constexpr double c = -0.3;

    expect_matrix_near(transforms::eul_to_R(a, b, c, "XYZ"), transforms::eul_to_R_extr(a, b, c, "XYZ"));
    expect_matrix_near(transforms::eul_to_C(a, b, c, "XYZ", "extr"), transforms::eul_to_C_extr(a, b, c, "XYZ"));
    expect_matrix_near(transforms::eul_to_C(a, b, c, "XYZ", "intr"), transforms::eul_to_C_intr(a, b, c, "XYZ"));
}

TEST(SO3Transforms, RejectsInvalidTypeArgument) {
    EXPECT_THROW(transforms::eul_to_C(0.0, 0.0, 0.0, "ZYX", "bad"), std::invalid_argument);
    EXPECT_THROW(transforms::C_to_eul(Eigen::Matrix3d::Identity(), "ZYX", "bad"), std::invalid_argument);
}

TEST(SO3Transforms, RejectsInvalidEulerOrderArgument) {
    EXPECT_THROW(transforms::eul_to_R(0.0, 0.0, 0.0, "BAD"), std::invalid_argument);
    EXPECT_THROW(transforms::eul_to_R_intr(0.0, 0.0, 0.0, "BAD"), std::invalid_argument);
    EXPECT_THROW(transforms::eul_to_C(0.0, 0.0, 0.0, "BAD", "extr"), std::invalid_argument);
    EXPECT_THROW(transforms::eul_to_C(0.0, 0.0, 0.0, "BAD", "intr"), std::invalid_argument);
}
