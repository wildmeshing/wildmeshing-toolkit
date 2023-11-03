#pragma once

#include <wmtk/image/Image.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <tuple>

namespace wmtk::internal {

template <typename T>
std::tuple<size_t, size_t, T> sample_coord(const T coord, const size_t size)
{
    const double xcoord = get_value(coord);
    assert(0.f <= xcoord && xcoord <= static_cast<double>(size));
    size_t coord0, coord1;
    T t(0.0);
    if (xcoord <= 0.5f) {
        coord0 = 0;
        coord1 = 0;
        t = T(0.5) + coord;
    } else if (xcoord + 0.5f >= static_cast<double>(size)) {
        coord0 = size - 1;
        coord1 = size - 1;
        t = coord - (T(coord0) + T(0.5));
    } else {
        assert(1 < size);
        coord0 = std::min(size - 2, static_cast<size_t>(xcoord - 0.5f));
        coord1 = coord0 + 1;
        t = coord - (T(coord0) + T(0.5));
    }
    return std::make_tuple(coord0, coord1, t);
};

inline uint32_t to_morton_z_order(uint16_t x, uint16_t y)
{
    constexpr auto swizzle = [](uint16_t x) {
        uint32_t z = x; // x &= 0x00'00'FF'FF
        z = (z ^ (z << 8)) & 0x00'FF'00'FF;
        z = (z ^ (z << 4)) & 0x0F'0F'0F'0F;
        z = (z ^ (z << 2)) & 0x33'33'33'33;
        z = (z ^ (z << 1)) & 0x55'55'55'55;
        return z;
    };
    auto z = (swizzle(y) << 1) + swizzle(x);
    return z;
}

inline float fetch_texel_offset(const wmtk::Image& image, int offset)
{
    return image.get_raw_image().data()[offset];
}

inline float fetch_texel_zorder(const wmtk::Image& image, int x, int y)
{
    auto zoffset = to_morton_z_order(x, y);
    return fetch_texel_offset(image, zoffset);
}

template <size_t N>
Eigen::Vector<float, N> fetch_texels_zorder(const std::array<wmtk::Image, N>& images, int x, int y)
{
    Eigen::Vector<float, N> res;
    auto zoffset = to_morton_z_order(x, y);
    for (size_t k = 0; k < N; ++k) {
        res[k] = fetch_texel_offset(images[k], zoffset);
    }
    return res;
}

inline float fetch_texel_eigen(const wmtk::Image& image, int x, int y)
{
    return image.get_raw_image()(y, x);
}

template <size_t N>
Eigen::Vector<float, N> fetch_texel_eigen(const std::array<wmtk::Image, N>& images, int x, int y)
{
    Eigen::Vector<float, N> res;
    for (size_t k = 0; k < N; ++k) {
        res[k] = fetch_texel_eigen(images[k], x, y);
    }
    return res;
}

inline float fetch_texel(const wmtk::Image& image, int x, int y)
{
    return fetch_texel_eigen(image, x, y);
}

template <size_t N>
Eigen::Vector<float, N> fetch_texels(const std::array<wmtk::Image, N>& images, int x, int y)
{
    return fetch_texel_eigen(images, x, y);
}

template <typename T, size_t N>
Eigen::Vector<T, N> sample_nearest(const std::array<wmtk::Image, N>& images, T u, T v)
{
    auto w = images[0].width();
    auto h = images[0].height();
    // x, y are between 0 and 1
    auto x = get_value(u) * static_cast<float>(w);
    auto y = get_value(v) * static_cast<float>(h);

    const auto sx = std::clamp(static_cast<int>(x), 0, w - 1);
    const auto sy = std::clamp(static_cast<int>(y), 0, h - 1);

    return fetch_texels(images, sx, sy).template cast<T>();
}

template <typename T, size_t N>
Eigen::Vector<T, N> sample_bilinear(const std::array<wmtk::Image, N>& images, T u, T v)
{
    auto w = images[0].width();
    auto h = images[0].height();
    // x, y are between 0 and 1
    auto x = u * static_cast<float>(w);
    auto y = v * static_cast<float>(h);

    const auto [x0, x1, tx] = sample_coord(x, w);
    const auto [y0, y1, ty] = sample_coord(y, h);

    const T _1(1.0);
    const Eigen::Vector4<T> weight((_1 - tx) * (_1 - ty), (_1 - tx) * ty, tx * (_1 - ty), tx * ty);

    Eigen::Vector<T, N> res;
    for (size_t k = 0; k < N; ++k) {
        Eigen::Vector4f pix(
            fetch_texel(images[k], x0, y0),
            fetch_texel(images[k], x0, y1),
            fetch_texel(images[k], x1, y0),
            fetch_texel(images[k], x1, y1));
        res[k] = pix.template cast<T>().dot(weight);
    }

    return res;
}

template <typename T, size_t N>
Eigen::Vector<T, N> sample_bicubic(const std::array<wmtk::Image, N>& images, T u, T v)
{
    auto w = images[0].width();
    auto h = images[0].height();
    // x, y are between 0 and 1
    auto x = u * static_cast<float>(w);
    auto y = v * static_cast<float>(h);

    const auto sx = static_cast<int>(std::floor(get_value(x) - 0.5));
    const auto sy = static_cast<int>(std::floor(get_value(y) - 0.5));

    struct vec2i
    {
        int x, y;
    };

    std::array<vec2i, 4> bicubic_taps = {
        vec2i{std::clamp(sx - 1, 0, w - 1), std::clamp(sy - 1, 0, h - 1)},
        vec2i{std::clamp(sx + 0, 0, w - 1), std::clamp(sy + 0, 0, h - 1)},
        vec2i{std::clamp(sx + 1, 0, w - 1), std::clamp(sy + 1, 0, h - 1)},
        vec2i{std::clamp(sx + 2, 0, w - 1), std::clamp(sy + 2, 0, h - 1)}};

    auto bicubic_samples = [](const wmtk::Image& image, const std::array<vec2i, 4>& c) {
        BicubicVector<float> samples;

        samples(0) = fetch_texel(image, c[0].x, c[0].y);
        samples(1) = fetch_texel(image, c[1].x, c[0].y);
        samples(2) = fetch_texel(image, c[2].x, c[0].y);
        samples(3) = fetch_texel(image, c[3].x, c[0].y);

        samples(4) = fetch_texel(image, c[0].x, c[1].y);
        samples(5) = fetch_texel(image, c[1].x, c[1].y);
        samples(6) = fetch_texel(image, c[2].x, c[1].y);
        samples(7) = fetch_texel(image, c[3].x, c[1].y);

        samples(8) = fetch_texel(image, c[0].x, c[2].y);
        samples(9) = fetch_texel(image, c[1].x, c[2].y);
        samples(10) = fetch_texel(image, c[2].x, c[2].y);
        samples(11) = fetch_texel(image, c[3].x, c[2].y);

        samples(12) = fetch_texel(image, c[0].x, c[3].y);
        samples(13) = fetch_texel(image, c[1].x, c[3].y);
        samples(14) = fetch_texel(image, c[2].x, c[3].y);
        samples(15) = fetch_texel(image, c[3].x, c[3].y);

        return samples;
    };

    // use bicubic interpolation
    Eigen::Vector<T, N> res;
    for (size_t k = 0; k < N; ++k) {
        BicubicVector<float> sample_vector = bicubic_samples(images[k], bicubic_taps);
        BicubicVector<float> bicubic_coeff = get_bicubic_matrix() * sample_vector;
        res[k] = eval_bicubic_coeffs(bicubic_coeff, x, y);
    }
    return res;
}

template <typename T, size_t N>
Eigen::Vector<T, N> sample_bicubic_old(const std::array<wmtk::Image, N>& images, T u, T v)
{
    auto w = images[0].width();
    auto h = images[0].height();
    // x, y are between 0 and 1
    auto x = u * static_cast<float>(w);
    auto y = v * static_cast<float>(h);

    // use bicubic interpolation
    Eigen::Vector<T, N> res;
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

template <size_t N>
inline std::array<wmtk::Image, N> convert_image_to_morton_z_order(
    const std::array<wmtk::Image, N>& linear_image)
{
    std::array<wmtk::Image, N> zorder_image;
    auto planes = linear_image.size();
    auto width = linear_image[0].width();
    auto height = linear_image[0].height();
    for (int k = 0; k < planes; ++k) {
        zorder_image[k] = wmtk::Image(height, width);
        auto&& zorder_plane = zorder_image[k].ref_raw_image().data();
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                auto zoffset = to_morton_z_order(x, y);
                zorder_plane[zoffset] = fetch_texel(linear_image[k], x, y);
            }
        }
    }
    return zorder_image;
}

} // namespace wmtk::internal
