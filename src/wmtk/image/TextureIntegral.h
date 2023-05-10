#pragma once

#include <lagrange/utils/span.h>
#include <lagrange/utils/value_ptr.h>
#include <wmtk/utils/Image.h>

#include <array>

namespace wmtk {

class TextureIntegral
{
public:
    TextureIntegral(std::array<wmtk::Image, 3> data);
    ~TextureIntegral();

    TextureIntegral(TextureIntegral&&) = delete;
    TextureIntegral& operator=(TextureIntegral&&) = delete;
    TextureIntegral(const TextureIntegral&) = delete;
    TextureIntegral& operator=(const TextureIntegral&) = delete;

    enum class SamplingMethod {
        Nearest,
        Bilinear,
        Bicubic,
    };

    enum class IntegrationMethod {
        Naive,
        Adaptive
    };

public:
    void set_sampling_method(SamplingMethod method) { m_sampling_method = method; }
    void set_integration_method(IntegrationMethod method) { m_integration_method = method; }

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
        lagrange::span<float> output_errors);

    ///
    /// Computes the integral of the input texture over each input UV triangle.
    ///
    /// Given a UV-triangle f=(v0, v1, v2), this function calculates the following integral:
    ///
    /// integral(f) = ∫_{u,v \in f} q_img(u, v) du dv
    ///
    /// @param[in]  input_triangles   UV coordinates of each input triangle corners (u0, v0, u1, v1,
    ///                               u2, v2).
    /// @param[in]  output_integrals  A pre-allocated buffer where to store the integral for each
    ///                               input triangle.
    ///
    void get_integral_per_triangle(
        lagrange::span<const std::array<float, 6>> input_triangles,
        lagrange::span<std::array<float, 3>> output_integrals);

protected:
    static float sample_nearest(const wmtk::Image& image, float u, float v);
    static float sample_bilinear(const wmtk::Image& image, float u, float v);
    static float sample_bicubic(const wmtk::Image& image, float u, float v);

    template<SamplingMethod sampling_method, IntegrationMethod integration_method>
    void get_error_per_triangle_internal(
        lagrange::span<const std::array<float, 6>> input_triangles,
        lagrange::span<float> output_errors);

private:
    struct Cache;
    std::array<wmtk::Image, 3> m_data;
    lagrange::value_ptr<Cache> m_cache;
    SamplingMethod m_sampling_method = SamplingMethod::Bicubic;
    IntegrationMethod m_integration_method = IntegrationMethod::Naive;
};

} // namespace wmtk
