#pragma once

#include <wmtk/utils/Image.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <tuple>

namespace wmtk::internal {

inline std::tuple<size_t, size_t, float> sample_coord(const float coord, const size_t size)
{
    assert(0.f <= coord && coord <= static_cast<float>(size));
    size_t coord0, coord1;
    float t;
    if (coord <= 0.5f) {
        coord0 = 0;
        coord1 = 0;
        t = 0.5f + coord;
    } else if (coord + 0.5f >= static_cast<float>(size)) {
        coord0 = size - 1;
        coord1 = size - 1;
        t = coord - (static_cast<float>(coord0) + 0.5f);
    } else {
        assert(1 < size);
        coord0 = std::min(size - 2, static_cast<size_t>(coord - 0.5f));
        coord1 = coord0 + 1;
        t = coord - (static_cast<float>(coord0) + 0.5f);
    }
    return std::make_tuple(coord0, coord1, t);
};

template <size_t N>
Eigen::Vector<float, N> sample_nearest(const std::array<wmtk::Image, N>& images, float u, float v)
{
    auto w = images[0].width();
    auto h = images[0].height();
    // x, y are between 0 and 1
    auto x = u * static_cast<float>(w);
    auto y = v * static_cast<float>(h);

    const auto sx = std::clamp(static_cast<int>(x), 0, w - 1);
    const auto sy = std::clamp(static_cast<int>(y), 0, h - 1);

    Eigen::Vector<float, N> res;
    for (size_t k = 0; k < N; ++k) {
        res[k] = images[k].get_raw_image()(sy, sx);
    }
    return res;
}

template <size_t N>
Eigen::Vector<float, N> sample_bilinear(const std::array<wmtk::Image, N>& images, float u, float v)
{
    auto w = images[0].width();
    auto h = images[0].height();
    // x, y are between 0 and 1
    auto x = u * static_cast<float>(w);
    auto y = v * static_cast<float>(h);

    const auto [x0, x1, tx] = sample_coord(x, w);
    const auto [y0, y1, ty] = sample_coord(y, h);

    const Eigen::Vector4f weight(
        (1.f - tx) * (1.f - ty),
        (1.f - tx) * ty,
        tx * (1.f - ty),
        tx * ty);

    Eigen::Vector<float, N> res;
    for (size_t k = 0; k < N; ++k) {
        const auto& array = images[k].get_raw_image();
        Eigen::Vector4f pix(array(y0, x0), array(y1, x0), array(y0, x1), array(y1, x1));
        res[k] = pix.dot(weight);
    }

    return res;
}

template <size_t N>
Eigen::Vector<float, N> sample_bicubic(const std::array<wmtk::Image, N>& images, float u, float v)
{
    auto w = images[0].width();
    auto h = images[0].height();
    // x, y are between 0 and 1
    auto x = u * static_cast<float>(w);
    auto y = v * static_cast<float>(h);

    // use bicubic interpolation
    Eigen::Vector<float, N> res;
    for (size_t k = 0; k < N; ++k) {
        BicubicVector<float> sample_vector = extract_samples(
            static_cast<size_t>(w),
            static_cast<size_t>(h),
            images[k].get_raw_image().data(),
            wmtk::get_value(x),
            wmtk::get_value(y),
            images[k].get_wrapping_mode_x(),
            images[k].get_wrapping_mode_y());
        BicubicVector<float> bicubic_coeff = get_bicubic_matrix() * sample_vector;
        res[k] = eval_bicubic_coeffs(bicubic_coeff, x, y);
    }
    return res;
}

// inline bool point_in_triangle(
//     const Eigen::Matrix<double, 3, 2, Eigen::RowMajor>& triangle,
//     const Eigen::Vector2d& point)
// {
//     Eigen::Vector2d v0 = triangle.row(0);
//     Eigen::Vector2d v1 = triangle.row(1);
//     Eigen::Vector2d v2 = triangle.row(2);
//     Eigen::Vector2d v0v1 = v1 - v0;
//     Eigen::Vector2d v0v2 = v2 - v0;
//     Eigen::Vector2d v0p = point - v0;

//     double dot00 = v0v2.dot(v0v2);
//     double dot01 = v0v2.dot(v0v1);
//     double dot02 = v0v2.dot(v0p);
//     double dot11 = v0v1.dot(v0v1);
//     double dot12 = v0v1.dot(v0p);

//     double inv_denom = 1 / (dot00 * dot11 - dot01 * dot01);
//     double u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
//     double v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

//     return (u >= 0) && (v >= 0) && (u + v < 1);
// }

inline bool point_in_triangle(
    const Eigen::Matrix<double, 3, 2, Eigen::RowMajor>& triangle,
    const Eigen::Vector2d& point)
{
    const auto& a = triangle.row(0);
    const auto& b = triangle.row(1);
    const auto& c = triangle.row(2);

    const double as_x = point.x() - a.x();
    const double as_y = point.y() - a.y();

    bool s_ab = (b.x() - a.x()) * as_y - (b.y() - a.y()) * as_x > 0;

    if ((c.x() - a.x()) * as_y - (c.y() - a.y()) * as_x > 0 == s_ab) {
        return false;
    }
    if ((c.x() - b.x()) * (point.y() - b.y()) - (c.y() - b.y()) * (point.x() - b.x()) > 0 != s_ab) {
        return false;
    }
    return true;
}

enum class Classification {
    Unknown,
    Inside,
    Outside,
};

inline Classification point_in_triangle_quick(
    const std::array<Eigen::Hyperplane<double, 2>, 3>& edges,
    const Eigen::Vector2d& point,
    double radius)
{
    Classification res = Classification::Inside;
    for (const auto& edge : edges) {
        const double s = edge.signedDistance(point);
        if (s < -radius) {
            return Classification::Outside;
        }
        if (s <= radius) {
            res = Classification::Unknown;
        }
    }
    return res;
}

inline Classification pixel_inside_triangle(
    const Eigen::Matrix<double, 3, 2, Eigen::RowMajor>& triangle,
    const Eigen::AlignedBox2d& pixel)
{
    for (int k = 0; k < 4; ++k) {
        bool inside = point_in_triangle(
            triangle,
            pixel.corner(static_cast<Eigen::AlignedBox2d::CornerType>(k)));
        if (!inside) {
            return Classification::Unknown;
        }
    }
    return Classification::Inside;
}

} // namespace wmtk::internal
