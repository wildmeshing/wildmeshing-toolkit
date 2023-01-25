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
    Image(int height_, int width_) { m_image.resize(height_, width_); };

public:
    // point coordinates between [0, 1]
    int width() const { return static_cast<int>(m_image.cols()); };
    int height() const { return static_cast<int>(m_image.rows()); };
    template <class T>
    std::decay_t<T> get(const T& u, const T& v) const;
    std::pair<size_t, size_t> get_raw(const double& u, const double& v) const;
    bool set(const std::function<float(const double&, const double&)>& f);
    bool save(const std::filesystem::path& path) const;
    void
    load(const std::filesystem::path& path, const WrappingMode mode_x, const WrappingMode mode_y);
    void set_wrapping_mode(WrappingMode mode_x, WrappingMode mode_y)
    {
        m_mode_x = mode_x;
        m_mode_y = mode_y;
    };
};

float modulo(double x, double n)
{
    float y = fmod(x, n);
    if (y < 0) {
        y += n;
    }
    return y;
}

unsigned char double_to_unsignedchar(const double d)
{
    return round(std::max(std::min(1., d), 0.) * 255);
}

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
        wmtk::get_floor_value(x),
        wmtk::get_floor_value(y),
        m_mode_x,
        m_mode_y);
    BicubicVector<float> bicubic_coeff = get_bicubic_matrix() * sample_vector;
    return eval_bicubic_coeffs(bicubic_coeff, x, y);
}

std::pair<size_t, size_t> Image::get_raw(const double& u, const double& v) const {
    int w = width();
    int h = height();
    auto size = std::max(w, h);
    // x, y are between 0 and 1
    auto x = u * static_cast<size_t>(size);
    auto y = v * static_cast<size_t>(size);

    return {x, y};
}

// set an image to have same value as the analytical function and save it to the file given
bool Image::set(const std::function<float(const double&, const double&)>& f)
{
    int h = height();
    int w = width();

    m_image.resize(h, w);

    for (int i = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            double u, v;
            u = (static_cast<double>(j) + 0.5) / static_cast<double>(w);
            v = (static_cast<double>(i) + 0.5) / static_cast<double>(h);
            m_image(i, j) = f(u, v);
        }
    }
    return true;
}

// save to hdr or exr
bool Image::save(const std::filesystem::path& path) const
{
    spdlog::trace("[save_image_hdr] start \"{}\"", path.string());
    int w = width();
    int h = height();
    std::vector<float> buffer;
    buffer.resize(w * h);

    for (auto i = 0; i < h; i++) {
        for (auto j = 0; j < w; j++) {
            buffer[i * w + j] = m_image(i, j);
        }
    }
    if (path.extension() == ".hdr") {
        auto res = stbi_write_hdr(path.string().c_str(), w, h, 1, buffer.data());
        assert(res);
    } else if (path.extension() == ".exr") {
        auto res = save_image_exr_red_channel(w, h, buffer, path);
    } else {
        spdlog::trace("[save_image_hdr] format doesn't support \"{}\"", path.string());
        return false;
    }

    spdlog::trace("[save_image] done \"{}\"", path.string());

    return true;
}

// load from hdr or exr
void Image::load(
    const std::filesystem::path& path,
    const WrappingMode mode_x,
    const WrappingMode mode_y)
{
    int w, h, channels;
    channels = 1;
    std::vector<float> buffer;
    if (path.extension() == ".exr") {
        std::tie(w, h, buffer) = load_image_exr_red_channel(path);
        assert(!buffer.empty());
    } else if (path.extension() == ".hdr") {
        auto res = stbi_loadf(path.string().c_str(), &w, &h, &channels, 1);
        buffer.assign(res, res + w * h);
    } else {
        spdlog::trace("[load_image] format doesn't support \"{}\"", path.string());
        return;
    }

    m_image.resize(w, h);

    for (int i = 0, k = 0; i < h; i++) {
        for (int j = 0; j < w; j++) {
            m_image(i, j) = buffer[k++];
        }
    }
    set_wrapping_mode(mode_x, mode_y);
}
} // namespace wmtk
