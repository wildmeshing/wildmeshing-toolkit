#include "QuadricIntegral.h"

#include <lagrange/utils/assert.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

namespace wmtk {

namespace {

// Column 0 is the center pixel, and other points are neighbors in clockwise order.
using Stencil = Eigen::Matrix<float, 3, 9>;

Stencil get_stencil(const std::array<wmtk::Image, 3>& images, int x, int y)
{
    auto w = images[0].width();
    auto h = images[0].height();

    auto get_point = [&](int sx, int sy) -> Eigen::Vector3f {
        sx = std::clamp(sx, 0, w - 1);
        sy = std::clamp(sy, 0, h - 1);
        return Eigen::Vector3f(
            images[0].get_raw_image()(sx, sy),
            images[1].get_raw_image()(sx, sy),
            images[2].get_raw_image()(sx, sy));
    };

    Stencil stencil;
    stencil.col(0) = get_point(x, y);
    stencil.col(1) = get_point(x - 1, y - 1);
    stencil.col(2) = get_point(x, y - 1);
    stencil.col(3) = get_point(x + 1, y - 1);
    stencil.col(4) = get_point(x + 1, y);
    stencil.col(5) = get_point(x + 1, y + 1);
    stencil.col(6) = get_point(x, y + 1);
    stencil.col(7) = get_point(x - 1, y + 1);
    stencil.col(8) = get_point(x - 1, y);
    return stencil;
}

double avg_distance_from_patch(const Stencil& patch)
{
    double dist = 0;
    for (int i = 1; i < 8; ++i) {
        const Eigen::Vector3d p0 = patch.col(0).cast<double>();
        const Eigen::Vector3d p1 = patch.col(i).cast<double>();
        dist += (p1 - p0).stableNorm();
    }
    return dist / 8;
}

Eigen::Vector3d normal_from_patch(const Stencil& patch)
{
    Eigen::Vector3d n = Eigen::Vector3d::Zero();
    for (int i = 1; i < 8; ++i) {
        const Eigen::Vector3d p0 = patch.col(0).cast<double>();
        const Eigen::Vector3d p1 = patch.col(i).cast<double>();
        const Eigen::Vector3d p2 = patch.col((i % 8) + 1).cast<double>();
        n += (p1 - p0).cross(p2 - p0);
    }
    return n.stableNormalized();
}

Quadric<double> compute_pixel_plane_quadric(const Stencil& patch, double sigma_q, double sigma_n)
{
    const auto mean_p = patch.col(0).cast<double>();
    const auto mean_n = normal_from_patch(patch);
    const double stddev_p = 0;
    const double stddev_n = 0.001;
    return Quadric<double>::probabilistic_plane_quadric(mean_p, mean_n, stddev_p, stddev_n);
}

Quadric<double> compute_pixel_triangle_quadric(const Stencil& patch, double sigma_q)
{
    Quadric<double> q;
    for (int i = 1; i < 8; ++i) {
        const Eigen::Vector3d p0 = patch.col(0).cast<double>();
        const Eigen::Vector3d p1 = patch.col(i).cast<double>();
        const Eigen::Vector3d p2 = patch.col((i % 8) + 1).cast<double>();
        q += Quadric<double>::probabilistic_triangle_quadric(p0, p1, p2, sigma_q);
    }
    return q;
}

void set_coefficients_from_quadric(
    std::array<wmtk::Image, 10>& coeffs,
    int x,
    int y,
    const Quadric<double>& q)
{
    coeffs[0].set(x, y, q.A00);
    coeffs[1].set(x, y, q.A01);
    coeffs[2].set(x, y, q.A02);
    coeffs[3].set(x, y, q.A11);
    coeffs[4].set(x, y, q.A12);
    coeffs[5].set(x, y, q.A22);
    coeffs[6].set(x, y, q.b0);
    coeffs[7].set(x, y, q.b1);
    coeffs[8].set(x, y, q.b2);
    coeffs[9].set(x, y, q.c);
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

    m_avg_pixel_distance = 0;
    for (int x = 0; x < w; ++x) {
        for (int y = 0; y < h; ++y) {
            auto stencil = get_stencil(displaced_positions, x, y);
            m_avg_pixel_distance += avg_distance_from_patch(stencil);
        }
    }
    m_avg_pixel_distance /= w * h;

    const double sigma_q = m_avg_pixel_distance * m_sigma_q_rel;

    tbb::parallel_for(0, w, [&](int x) {
        for (int y = 0; y < h; ++y) {
            auto stencil = get_stencil(displaced_positions, x, y);
            if (m_quadric_type == QuadricType::Plane) {
                auto quadric = compute_pixel_plane_quadric(stencil, sigma_q, m_sigma_n);
                set_coefficients_from_quadric(m_quadrics, x, y, quadric);
            } else {
                auto quadric = compute_pixel_triangle_quadric(stencil, sigma_q);
                set_coefficients_from_quadric(m_quadrics, x, y, quadric);
            }
        }
    });
}

void QuadricIntegral::get_quadric_per_triangle(
    int num_triangles,
    lagrange::function_ref<std::array<float, 6>(int)> get_triangle,
    lagrange::span<wmtk::Quadric<double>> output_quadrics)
{}

} // namespace wmtk
