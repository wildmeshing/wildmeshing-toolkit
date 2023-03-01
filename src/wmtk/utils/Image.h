#pragma once

#include <stb_image.h>
#include <stb_image_write.h>
#include <Eigen/Core>
#include <array>
#include <cmath>
#include <filesystem>
#include <type_traits>
#include "bicubic_interpolation.h"
#include "load_image_exr.h"
#include "save_image_exr.h"

namespace wmtk {
class Image
{
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;

protected:
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        m_image; // saving scanline images
    WrappingMode m_mode_x = WrappingMode::CLAMP_TO_EDGE;
    WrappingMode m_mode_y = WrappingMode::CLAMP_TO_EDGE;

public:
    Image() = default;
    Image(int height_, int width_) { m_image.resize(height_, width_); };

    auto get_raw_image() const { return m_image; }

public:
    // point coordinates between [0, 1]
    int width() const { return static_cast<int>(m_image.cols()); };
    int height() const { return static_cast<int>(m_image.rows()); };

    template <class T>
    std::decay_t<T> get(const T& u, const T& v) const;
    std::pair<int, int> get_pixel_index(const double& u, const double& v) const;
    int get_coordinate(const int x, const WrappingMode mode) const;
    WrappingMode get_wrapping_mode_x() const { return m_mode_x; };
    WrappingMode get_wrapping_mode_y() const { return m_mode_y; };
    bool set(
        const std::function<float(const double&, const double&)>& f,
        const WrappingMode mode_x = WrappingMode::CLAMP_TO_EDGE,
        const WrappingMode mode_y = WrappingMode::CLAMP_TO_EDGE);
    bool set(const int r, const int c, const float v)
    {
        m_image(r, c) = v;
        return true;
    };
    bool save(const std::filesystem::path& path) const;
    void load(const std::filesystem::path& path, WrappingMode mode_x, WrappingMode mode_y);

    void set_wrapping_mode(WrappingMode mode_x, WrappingMode mode_y)
    {
        m_mode_x = mode_x;
        m_mode_y = mode_y;
    };
    Image down_sample() const;
};

/// @brief
/// @param p coordinates between (0,1)
/// @return /
template <class T>
std::decay_t<T> Image::get(const T& u, const T& v) const
{
    int w = width();
    int h = height();
    auto size = std::max(w, h);
    // x, y are between 0 and 1
    auto x = u * static_cast<std::decay_t<T>>(size);
    auto y = v * static_cast<std::decay_t<T>>(size);
    // use bicubic interpolation

    BicubicVector<float> sample_vector = extract_samples(
        static_cast<size_t>(w),
        static_cast<size_t>(h),
        m_image.data(),
        wmtk::get_value(x),
        wmtk::get_value(y),
        m_mode_x,
        m_mode_y);
    BicubicVector<float> bicubic_coeff = get_bicubic_matrix() * sample_vector;
    return eval_bicubic_coeffs(bicubic_coeff, x, y);
}
} // namespace wmtk
