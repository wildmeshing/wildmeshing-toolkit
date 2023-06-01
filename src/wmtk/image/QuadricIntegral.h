#pragma once

#include "IntegralBase.h"

#include <lagrange/utils/function_ref.h>
#include <lagrange/utils/span.h>
#include <lagrange/utils/value_ptr.h>
#include <wmtk/image/Image.h>
#include <wmtk/image/Quadric.h>

#include <array>

namespace wmtk {

class QuadricIntegral : public IntegralBase
{
public:
    enum class QuadricType {
        Point,
        Plane,
        Triangle,
    };

    ///
    /// Constructs an acceleration structure to efficiently compute the integral of a quadric error
    /// function encoding the distance from a point to the displaced surface in a triangular region.
    ///
    /// @param[in]  displaced_positions  Displaced position texture.
    /// @param[in]  quadric_type         Quadric type to use.
    ///
    QuadricIntegral(
        const std::array<wmtk::Image, 3>& displaced_positions,
        QuadricType quadric_type);

    QuadricIntegral();
    ~QuadricIntegral();

    QuadricIntegral(QuadricIntegral&&);
    QuadricIntegral& operator=(QuadricIntegral&&);
    QuadricIntegral(const QuadricIntegral&) = delete;
    QuadricIntegral& operator=(const QuadricIntegral&) = delete;

public:
    ///
    /// Computes the quadric integrated over each input UV triangle.
    ///
    /// @param[in]  num_triangles    Number of UV triangle in the input.
    /// @param[in]  get_triangle     A thread-safe function for retrieving the UV coordinates of
    ///                              each input triangle corners (u0, v0, u1, v1, u2, v2).
    /// @param[in]  output_quadrics  A pre-allocated buffer where to store the quadric for each
    ///                              input triangle. Output quadrics are weighted by the facet uv
    ///                              area.
    ///
    void get_quadric_per_triangle(
        int num_triangles,
        lagrange::function_ref<std::array<float, 6>(int)> get_triangle,
        lagrange::span<wmtk::Quadric<double>> output_quadrics) const;

protected:
    template <SamplingMethod sampling_method, IntegrationMethod integration_method>
    void get_quadric_per_triangle_internal(
        int num_triangles,
        lagrange::function_ref<std::array<float, 6>(int)> get_triangle,
        lagrange::span<wmtk::Quadric<double>> output_quadrics,
        int order) const;

protected:
    struct Cache;

    // Quadrics coefficients
    std::array<wmtk::Image, 10> m_quadrics;

    // Hidden cache data
    lagrange::value_ptr<Cache> m_cache;
};

} // namespace wmtk
