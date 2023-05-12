#include "QuadricIntegral.h"

#include <lagrange/utils/assert.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

namespace wmtk {

namespace {

using Stencil = Eigen::Matrix3<Eigen::Vector3f>;

Stencil get_stencil(const std::array<wmtk::Image, 3>& images, int x, int y)
{
    auto w = images[0].width();
    auto h = images[0].height();
    Stencil stencil;
    for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
            const auto sx = std::clamp(x + dx, 0, w - 1);
            const auto sy = std::clamp(y + dy, 0, h - 1);
            for (size_t k = 0; k < 3; ++k) {
                stencil(dx + 1, dy + 1)(k) = images[k].get_raw_image()(x + dx, y + dy);
            }
        }
    }
    return stencil;
}

Quadric<double> compute_pixel_plane_quadric(const Stencil& patch)
{
    return {};
}

Quadric<double> compute_pixel_triangle_quadric(const Stencil& patch)
{
    return {};
}

} // namespace

struct QuadricIntegral::Cache
{
};

QuadricIntegral::~QuadricIntegral() = default;

QuadricIntegral::QuadricIntegral(const std::array<wmtk::Image, 3>& displaced_positions)
{
    auto w = displaced_positions[0].width();
    auto h = displaced_positions[0].height();
    // tbb::parallel_for(0, w, [&](int x) {
    //     for (int y = 0; y < h; ++y) {
    //         auto stencil = get_stencil(displaced_positions, x, y);
    //         auto quadric = compute_pixel_plane_quadric(stencil);
    //         m_quadrics(x, y) = quadric;
    //     }
    // });
}

void QuadricIntegral::get_quadric_per_triangle(
    lagrange::span<const std::array<float, 6>> input_triangles,
    lagrange::span<Quadric<double>> output_quadrics)
{}

} // namespace wmtk
