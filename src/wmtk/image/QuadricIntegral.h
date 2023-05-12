#pragma once

#include <lagrange/utils/function_ref.h>
#include <lagrange/utils/span.h>
#include <lagrange/utils/value_ptr.h>
#include <wmtk/utils/Image.h>
#include <wmtk/utils/Quadric.h>

#include <array>

namespace wmtk {

class QuadricIntegral
{
public:
    ///
    /// Constructs an acceleration structure to efficiently compute the integral of a quadric error
    /// function encoding the distance from a point to the displaced surface in a triangular region.
    ///
    /// @param[in]  displaced_positions  Displaced position texture.
    ///
    QuadricIntegral(const std::array<wmtk::Image, 3>& displaced_positions);

    ~QuadricIntegral();

    QuadricIntegral(QuadricIntegral&&) = delete;
    QuadricIntegral& operator=(QuadricIntegral&&) = delete;
    QuadricIntegral(const QuadricIntegral&) = delete;
    QuadricIntegral& operator=(const QuadricIntegral&) = delete;

    enum class QuadricType {
        Plane,
        Triangle,
    };

public:
    ///
    /// Computes the quadric integrated over each input UV triangle.
    ///
    /// @param[in]  num_triangles    Number of UV triangle in the input.
    /// @param[in]  get_triangle     A thread-safe function for retrieving the UV coordinates of
    ///                              each input triangle corners (u0, v0, u1, v1, u2, v2).
    /// @param[in]  output_quadrics  A pre-allocated buffer where to store the quadric for each
    ///                              input triangle.
    ///
    void get_quadric_per_triangle(
        int num_triangles,
        lagrange::function_ref<std::array<float, 6>(int)> get_triangle,
        lagrange::span<wmtk::Quadric<double>> output_quadrics);

private:
    struct Cache;

    // Quadrics coefficients
    std::array<wmtk::Image, 10> m_quadrics;

    // Hidden cache data
    lagrange::value_ptr<Cache> m_cache;

    // Average distance between adjacent pixels in the input image
    double m_avg_pixel_distance = 1.0;

    // Uncertainty on point positions (relative to the mean distance between adjacent pixels in the
    // input image)
    double m_sigma_q_rel = 1e-4;

    // Uncertainty on normal directions
    double m_sigma_n = 0.001;

    // Type of quadric to use
    QuadricType m_quadric_type = QuadricType::Plane;
};

} // namespace wmtk
