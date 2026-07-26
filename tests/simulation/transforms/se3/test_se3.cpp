#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <stdexcept>
#include <vector>

#include "simulation/transforms/se3/public.hpp"
#include "simulation/transforms/se3/private.hpp"
#include "simulation/transforms/so3/public.hpp"
#include "simulation/constants/public.hpp"

static void expect_matrix4_near(const Eigen::Matrix4d& A, const Eigen::Matrix4d& B) {
    EXPECT_TRUE(A.isApprox(B, constants::eps_strict));
}

static void expect_matrix3_near(const Eigen::Matrix3d& A, const Eigen::Matrix3d& B) {
    EXPECT_TRUE(A.isApprox(B, constants::eps_strict));
}

static void expect_vector_near(const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
    EXPECT_TRUE(a.isApprox(b, constants::eps_strict));
}

static void expect_valid_homogeneous_matrix(const Eigen::Matrix4d& H) {
    EXPECT_TRUE(H.row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), constants::eps_strict));
    EXPECT_TRUE((H.block<3,3>(0,0).transpose() * H.block<3,3>(0,0)).isApprox(constants::IX_T<double, 3>, constants::eps_strict));
    EXPECT_NEAR((H.block<3,3>(0,0).determinant()), 1.0, constants::eps_strict);
}

TEST(transforms_se3, MakeHRBuildsRotateAndTranslateFirstTransforms) {
    const auto R = transforms::eul_to_R(0.2, -0.3, 0.4, transforms::EulerOrder::ZYX);
    const Eigen::Vector3d d(1.0, -2.0, 0.5);
    const Eigen::Vector3d v(0.25, 0.5, -0.75);

    const auto H_rotate = transforms::make_HR(R, d, transforms::TransformationOrder::RotateFirst);
    expect_matrix3_near(transforms::R_from_H(H_rotate), R);
    expect_vector_near(transforms::d_from_H(H_rotate), d);
    expect_vector_near(transforms::apply_H(H_rotate, v), R * v + d);
    expect_valid_homogeneous_matrix(H_rotate);

    const auto H_translate = transforms::make_HR(R, d, transforms::TransformationOrder::TranslateFirst);
    expect_matrix3_near(transforms::R_from_H(H_translate), R);
    expect_vector_near(transforms::d_from_H(H_translate), R * d);
    expect_vector_near(transforms::apply_H(H_translate, v), R * (v + d));
    expect_valid_homogeneous_matrix(H_translate);
}

TEST(transforms_se3, MakeHCBuildsRotateAndTranslateFirstTransforms) {
    const auto R = transforms::eul_to_R(0.2, -0.3, 0.4, transforms::EulerOrder::ZYX);
    const auto C = R.transpose();
    const Eigen::Vector3d d(1.0, -2.0, 0.5);
    const Eigen::Vector3d v(0.25, 0.5, -0.75);

    const auto H_rotate = transforms::make_HC(C, d, transforms::TransformationOrder::RotateFirst);
    expect_matrix3_near(transforms::C_from_H(H_rotate), C);
    expect_vector_near(transforms::d_from_H(H_rotate), -d);
    expect_vector_near(transforms::apply_H(H_rotate, v), C * v - d);
    expect_valid_homogeneous_matrix(H_rotate);

    const auto H_translate = transforms::make_HC(C, d, transforms::TransformationOrder::TranslateFirst);
    expect_matrix3_near(transforms::C_from_H(H_translate), C);
    expect_vector_near(transforms::d_from_H(H_translate), -C * d);
    expect_vector_near(transforms::p_from_H(H_translate), d);
    expect_vector_near(transforms::apply_H(H_translate, v), C * (v - d));
    expect_valid_homogeneous_matrix(H_translate);
}

TEST(transforms_se3, MakeHinvInvertsHomogeneousTransform) {
    const auto R = transforms::eul_to_R(0.2, -0.3, 0.4, transforms::EulerOrder::ZYX);
    const auto C = R.transpose();
    const Eigen::Vector3d d(1.0, -2.0, 0.5);
    const Eigen::Vector3d v(0.25, 0.5, -0.75);
    const std::vector<Eigen::Matrix4d> H_list = {
        transforms::make_HR(R, d, transforms::TransformationOrder::RotateFirst),
        transforms::make_HR(R, d, transforms::TransformationOrder::TranslateFirst),
        transforms::make_HC(C, d, transforms::TransformationOrder::RotateFirst),
        transforms::make_HC(C, d, transforms::TransformationOrder::TranslateFirst)
    };

    for (const auto& H : H_list) {
        const auto H_inv = transforms::make_Hinv(H);

        expect_matrix4_near(H * H_inv, constants::HI);
        expect_matrix4_near(H_inv * H, constants::HI);
        expect_vector_near(transforms::apply_H(H_inv, transforms::apply_H(H, v)), v);
    }
}

TEST(transforms_se3, ChainHomPostAndPreComposeInExpectedOrder) {
    const auto R1 = transforms::eul_to_R(0.2, -0.3, 0.4, transforms::EulerOrder::ZYX);
    const auto R2 = transforms::eul_to_R(-0.1, 0.5, 0.2, transforms::EulerOrder::XYZ);
    const auto C1 = R1.transpose();
    const auto C2 = R2.transpose();
    const Eigen::Vector3d d1(1.0, -2.0, 0.5);
    const Eigen::Vector3d d2(-0.25, 0.75, 1.5);

    const auto expect_chain_order = [](const Eigen::Matrix4d& H1, const Eigen::Matrix4d& H2) {
        const std::vector<Eigen::Matrix4d> H_list = {H1, H2};
        expect_matrix4_near(transforms::chain_hom_post(H_list), H1 * H2);
        expect_matrix4_near(transforms::chain_hom_pre(H_list), H2 * H1);
    };

    expect_chain_order(transforms::make_HR(R1, d1, transforms::TransformationOrder::RotateFirst), transforms::make_HR(R2, d2, transforms::TransformationOrder::RotateFirst));
    expect_chain_order(transforms::make_HR(R1, d1, transforms::TransformationOrder::TranslateFirst), transforms::make_HR(R2, d2, transforms::TransformationOrder::TranslateFirst));
    expect_chain_order(transforms::make_HC(C1, d1, transforms::TransformationOrder::RotateFirst), transforms::make_HC(C2, d2, transforms::TransformationOrder::RotateFirst));
    expect_chain_order(transforms::make_HC(C1, d1, transforms::TransformationOrder::TranslateFirst), transforms::make_HC(C2, d2, transforms::TransformationOrder::TranslateFirst));
}

TEST(transforms_se3, MakeHWrappersDispatchToRotateAndTranslateFirst) {
    const auto R = transforms::eul_to_R(0.2, -0.3, 0.4, transforms::EulerOrder::ZYX);
    const auto C = R.transpose();
    const Eigen::Vector3d d(1.0, -2.0, 0.5);

    expect_matrix4_near(transforms::make_HR(R, d, transforms::TransformationOrder::RotateFirst), transforms::make_HR_rotate_first(R, d));
    expect_matrix4_near(transforms::make_HR(R, d, transforms::TransformationOrder::TranslateFirst), transforms::make_HR_translate_first(R, d));
    expect_matrix4_near(transforms::make_HC(C, d, transforms::TransformationOrder::RotateFirst), transforms::make_HC_rotate_first(C, d));
    expect_matrix4_near(transforms::make_HC(C, d, transforms::TransformationOrder::TranslateFirst), transforms::make_HC_translate_first(C, d));
}

TEST(transforms_se3, RejectsInvalidTransformationOrderArgument) {
    const auto R = transforms::eul_to_R(0.2, -0.3, 0.4, transforms::EulerOrder::ZYX);
    const auto C = R.transpose();
    const Eigen::Vector3d d(1.0, -2.0, 0.5);

    EXPECT_THROW(transforms::make_HR(R, d, static_cast<transforms::TransformationOrder>(-1)), std::invalid_argument);
    EXPECT_THROW(transforms::make_HC(C, d, static_cast<transforms::TransformationOrder>(-1)), std::invalid_argument);
}
