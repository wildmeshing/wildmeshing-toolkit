#pragma once

#include "IntegralBase.h"

#include <lagrange/utils/span.h>
#include <lagrange/utils/value_ptr.h>
#include <wmtk/utils/Image.h>
#include <wmtk/utils/autodiff.h>

#include <array>

namespace wmtk {

class TextureIntegral : public IntegralBase
{
public:
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;
    using DTriangle = Eigen::Matrix<DScalar, 3, 2, Eigen::RowMajor>;

public:
    TextureIntegral(); // default constructor
    TextureIntegral(const TextureIntegral&) = delete; // copy constructor
    TextureIntegral(TextureIntegral&&); // move constructor
    TextureIntegral& operator=(const TextureIntegral&) = delete; // copy assignment operator
    TextureIntegral& operator=(TextureIntegral&&); // move assignment operator
    ~TextureIntegral(); // destructor

    TextureIntegral(std::array<wmtk::Image, 3> data);

public:
    ///
    /// Computes the error integral per triangle.
    ///
    /// Let q_img(.) be a texture mapping UV-space to the displaced positions in R^3.
    ///
    /// Given a UV-triangle f=(v0, v1, v2), a point m=(u, v) inside f will be mapped to the 3D
    /// position
    ///
    /// q_tri(u, v) = λ * q_v0 + μ * q_v1 + (1 - λ - μ) * q_v2
    ///
    /// where:
    /// - λ and μ are the barycentric coordinates of m inside the triangle f.
    /// - q_vi are the 3D positions of the triangle corners v0, v1, v2 according the q_img(.)
    ///   formula above.
    ///
    /// The "error" function is then defined as the integral of the distance between q_img(u, v) and
    /// q_tri(u, v) over the triangle f:
    ///
    /// error(f) = ∫_{u,v \in f} ‖q_tri(u, v) - q_img(u, v)‖^2 du dv
    ///
    /// @param[in]  input_triangles  UV coordinates of each input triangle corners (u0, v0, u1, v1,
    ///                              u2, v2).
    /// @param[in]  output_errors    A pre-allocated buffer where to store the error integral for
    ///                              each input triangle.
    ///
    void get_error_per_triangle(
        lagrange::span<const std::array<float, 6>> input_triangles,
        lagrange::span<float> output_errors) const;

    /// Same as above, but computes an autodiffable error for a single triangle.
    DScalar get_error_one_triangle(const DTriangle& input_triangle) const;

protected:
    template <SamplingMethod sampling_method, IntegrationMethod integration_method>
    void get_error_per_triangle_internal(
        lagrange::span<const std::array<float, 6>> input_triangles,
        lagrange::span<float> output_errors,
        int order) const;

    template <SamplingMethod sampling_method, IntegrationMethod integration_method>
    DScalar get_error_one_triangle_internal(const DTriangle& input_triangle, int order) const;

protected:
    struct Cache;
    std::array<wmtk::Image, 3> m_data;
    lagrange::value_ptr<Cache> m_cache;
};

} // namespace wmtk
