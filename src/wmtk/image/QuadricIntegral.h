#pragma once

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

public:
    ///
    /// Computes the quadric integrated over each input UV triangle.
    ///
    /// @param[in]  input_triangles   UV coordinates of each input triangle corners (u0, v0, u1, v1,
    ///                               u2, v2).
    /// @param[in]  output_quadrics   A pre-allocated buffer where to store the quadric for each
    ///                               input triangle.
    ///
    void get_quadric_per_triangle(
        lagrange::span<const std::array<float, 6>> input_triangles,
        lagrange::span<wmtk::Quadric<double>> output_quadrics);

private:
    struct Cache;

    // Quadrics coefficients
    std::array<wmtk::Image, 10> m_quadrics;

    // Hidden cache data
    lagrange::value_ptr<Cache> m_cache;
};

} // namespace wmtk
