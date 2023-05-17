#pragma once

#include <wmtk/utils/Image.h>

#include <Eigen/Core>

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

inline float fetch_texel(const wmtk::Image& image, int offset)
{
    return image.get_raw_image().data()[offset];
}

inline float fetch_texel(const wmtk::Image& image, int x, int y)
{
    return image.get_raw_image()(x, y);
}

template <size_t N>
Eigen::Vector<float, N> fetch_texels(const std::array<wmtk::Image, N>& images, int x, int y)
{
    Eigen::Vector<float, N> res;
    for (size_t k = 0; k < N; ++k) {
        res[k] = fetch_texel(images[k], x, y);
    }
    return res;
}

template <size_t N>
Eigen::Vector<float, N> fetch_texels_zorder(const std::array<wmtk::Image, N>& images, int x, int y)
{
    Eigen::Vector<float, N> res;
    auto zoffset = to_morton_z_order(x, y);
    for (size_t k = 0; k < N; ++k) {
        res[k] = fetch_texel(images[k], zoffset);
    }
    return res;
}

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

    return fetch_texels(images, sx, sy);
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
        tx * (1.f - ty),
        (1.f - tx) * ty,
        tx * ty);

    Eigen::Vector<float, N> res;
    for (size_t k = 0; k < N; ++k) {
        Eigen::Vector4f pix(fetch_texel(images[k], x0, y0),
                            fetch_texel(images[k], x1, y0),
                            fetch_texel(images[k], x0, y1),
                            fetch_texel(images[k], x1, y1));
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

template <size_t N>
inline std::array<wmtk::Image, N> convert_image_to_morton_z_order(const std::array<wmtk::Image, N>& linear_image)
{
    auto zorder_image = linear_image;
    auto num_planes = linear_image.size();
    auto width = linear_image[0].width();
    auto height = linear_image[0].height();
    for (int k = 0; k < num_planes; ++k)
    {
        for (int y = 0; y < height; ++y)
        {
            for (int x = 0; x < width; ++x)
            {
                auto zoffset = wmtk::internal::to_morton_z_order(x, y);
                zorder_image[k].get_raw_image().data()[zoffset] = linear_image[k].get_raw_image().coeff(x, y);
            }
        }
    }
    return zorder_image;
}

} // namespace wmtk::internal
