#pragma once

#include <wmtk/components/adaptive_tessellation/image/Image.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <tuple>
#include "SamplingParameters.hpp"
#include "bicubic_interpolation.hpp"
enum class Classification {
    Unknown,
    Inside,
    Outside,
};

template <class T>
using BicubicVector = Eigen::Matrix<T, 16, 1>;
using BicubicMatrix = Eigen::Matrix<float, 16, 16>;
using namespace Eigen;
typedef DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> DScalar;
namespace wmtk::components::adaptive_tessellation::image {
using DScalar = DScalar2<double, Eigen::Matrix<double, -1, 1>, Eigen::Matrix<double, -1, -1>>;
inline double get_double(float x)
{
    return static_cast<double>(x);
}

inline double get_double(double x)
{
    return x;
}

inline double get_double(DScalar x)
{
    return x.getValue();
}

template <typename T>
inline Vector2d get_double(const Vector2<T>& vector_x);

template <>
inline Vector2d get_double(const Vector2<DScalar>& vector_x)
{
    VectorXd res(vector_x.size());
    for (int i = 0; i < vector_x.size(); ++i) {
        res(i) = vector_x(i).getValue();
    }
    return res;
}
template <>
inline Vector2d get_double(const Vector2<double>& vector_x)
{
    return vector_x;
}


int get_pixel_index_with_image_wrapping_mode(
    const int x,
    const int width,
    const int height,
    const IMAGE_WRAPPING_MODE mode);

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

uint32_t to_morton_z_order(uint16_t x, uint16_t y);

float fetch_texel_offset(const Image& image, int offset);

float fetch_texel_zorder(const Image& image, int x, int y);

template <size_t N>
Eigen::Vector<float, N> fetch_texels_zorder(const std::array<Image, N>& images, int x, int y)
{
    Eigen::Vector<float, N> res;
    auto zoffset = to_morton_z_order(x, y);
    for (size_t k = 0; k < N; ++k) {
        res[k] = fetch_texel_offset(images[k], zoffset);
    }
    return res;
}

float fetch_texel_eigen(const Image& image, int x, int y);
template <size_t N>
Eigen::Vector<float, N> fetch_texel_eigen(const std::array<Image, N>& images, int x, int y)
{
    Eigen::Vector<float, N> res;
    for (size_t k = 0; k < N; ++k) {
        res[k] = fetch_texel_eigen(images[k], x, y);
    }
    return res;
}

float fetch_texel(const Image& image, int x, int y);
template <size_t N>
Eigen::Vector<float, N> fetch_texels(const std::array<Image, N>& images, int x, int y)
{
    return fetch_texel_eigen(images, x, y);
}

template <typename T>
T sample_nearest(const Image& image, T u, T v)
{
    auto w = image.width();
    auto h = image.height();
    // x, y are between 0 and 1
    auto x = get_value(u) * static_cast<float>(w);
    auto y = get_value(v) * static_cast<float>(h);

    const auto sx = std::clamp(static_cast<int>(x), 0, w - 1);
    const auto sy = std::clamp(static_cast<int>(y), 0, h - 1);
    return static_cast<T>(fetch_texel(image, sx, sy));
}
template <typename T, size_t N>
Eigen::Vector<T, N> sample_nearest(const std::array<std::shared_ptr<Image>, N>& images, T u, T v)
{
    // use bicubic interpolation
    Eigen::Vector<T, N> res;
    for (size_t k = 0; k < N; ++k) {
        res[k] = sample_nearest(*images[k], u, v);
    }
    return res;
}

template <typename T>
T sample_bilinear(const Image& image, T u, T v)
{
    auto w = image.width();
    auto h = image.height();
    // x, y are between 0 and 1
    auto x = u * static_cast<float>(w);
    auto y = v * static_cast<float>(h);

    const auto [x0, x1, tx] = sample_coord(x, w);
    const auto [y0, y1, ty] = sample_coord(y, h);

    const T _1(1.0);
    const Eigen::Vector4<T> weight((_1 - tx) * (_1 - ty), (_1 - tx) * ty, tx * (_1 - ty), tx * ty);
    Eigen::Vector4f pix(
        fetch_texel(image, x0, y0),
        fetch_texel(image, x0, y1),
        fetch_texel(image, x1, y0),
        fetch_texel(image, x1, y1));
    return pix.template cast<T>().dot(weight);
}

template <typename T, size_t N>
Eigen::Vector<T, N> sample_bilinear(const std::array<std::shared_ptr<Image>, N>& images, T u, T v)
{
    Eigen::Vector<T, N> res;
    for (size_t k = 0; k < N; ++k) {
        res[k] = sample_bilinear(*images[k], u, v);
    }
    return res;
}

template <typename T>
T sample_bicubic(const Image& image, T u, T v)
{
    auto w = image.width();
    auto h = image.height();
    // x, y are between 0 and 1
    auto x = u * static_cast<float>(w);
    auto y = v * static_cast<float>(h);

    const int sx = static_cast<int>(std::floor(get_value(x) - 0.5));
    const int sy = static_cast<int>(std::floor(get_value(y) - 0.5));

    struct vec2i
    {
        int x, y;
    };

    std::array<vec2i, 4> bicubic_taps = {
        {vec2i{std::clamp(sx - 1, 0, w - 1), std::clamp(sy - 1, 0, h - 1)},
         vec2i{std::clamp(sx + 0, 0, w - 1), std::clamp(sy + 0, 0, h - 1)},
         vec2i{std::clamp(sx + 1, 0, w - 1), std::clamp(sy + 1, 0, h - 1)},
         vec2i{std::clamp(sx + 2, 0, w - 1), std::clamp(sy + 2, 0, h - 1)}}};

    auto bicubic_samples = [&image](const std::array<vec2i, 4>& c) {
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

    BicubicVector<float> sample_vector = bicubic_samples(bicubic_taps);
    BicubicVector<float> bicubic_coeff = get_bicubic_matrix() * sample_vector;
    return eval_bicubic_coeffs(bicubic_coeff, x, y);
}

template <typename T, size_t N>
Eigen::Vector<T, N> sample_bicubic(const std::array<std::shared_ptr<Image>, N>& images, T u, T v)
{
    // use bicubic interpolation
    Eigen::Vector<T, N> res;
    for (size_t k = 0; k < N; ++k) {
        res[k] = sample_bicubic(*images[k], u, v);
    }
    return res;
}

bool point_in_triangle(
    const Eigen::Matrix<double, 3, 2, Eigen::RowMajor>& triangle,
    const Eigen::Vector2d& point);

Classification point_in_triangle_quick(
    const std::array<Eigen::Hyperplane<double, 2>, 3>& edges,
    const Eigen::Vector2d& point,
    double radius);

Classification pixel_inside_triangle(
    const Eigen::Matrix<double, 3, 2, Eigen::RowMajor>& triangle,
    const Eigen::AlignedBox2d& pixel);

template <size_t N>
inline std::array<Image, N> convert_image_to_morton_z_order(
    const std::array<Image, N>& linear_image)
{
    std::array<Image, N> zorder_image;
    auto planes = linear_image.size();
    auto width = linear_image[0].width();
    auto height = linear_image[0].height();
    for (int k = 0; k < planes; ++k) {
        zorder_image[k] = Image(height, width);
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

} // namespace wmtk::components::adaptive_tessellation::image
