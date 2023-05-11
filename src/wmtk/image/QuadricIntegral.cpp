#include "QuadricIntegral.h"

#include <lagrange/utils/assert.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

namespace wmtk {

namespace {

// Get a 3x3 stencil around a given u,v point
void get_stencil() {}

void compute_pixel_plane_quadric() {}

void compute_pixel_triangle_quadric() {}

} // namespace

struct QuadricIntegral::Cache
{
};

QuadricIntegral::~QuadricIntegral() = default;

QuadricIntegral::QuadricIntegral(std::array<wmtk::Image, 3> displaced_positions)
    : m_data(displaced_positions)
{}

void QuadricIntegral::get_quadric_per_triangle(
    lagrange::span<const std::array<float, 6>> input_triangles,
    lagrange::span<Quadric<double>> output_quadrics)
{}

} // namespace wmtk
