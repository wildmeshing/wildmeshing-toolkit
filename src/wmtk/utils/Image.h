#pragma once

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <filesystem>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>
#include "bicubic_interpolation.h"
#include "load_image_exr.h"
#include "save_image_exr.h"
namespace wmtk {
class Image
{
protected:
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        m_image; // saving scanline images
    WrappingMode m_mode_x;
    WrappingMode m_mode_y;

public:
    Image(int height_, int width_) { m_image.resize(height_, width_); };

public:
    // point coordinates between [0, 1]
    int width() const { return static_cast<int>(m_image.cols()); };
    int height() const { return static_cast<int>(m_image.rows()); };
    float get(const Eigen::Vector2d& p) const;
    float get_raw(const Eigen::Vector2i& index) const { return m_image(index.y(), index.x()); };
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

unsigned char unsignedchar_to_double(const unsigned char c){};

float Image::get(const Eigen::Vector2d& p) const
{
    int w = width();
    int h = height();
    auto size = std::max(w, h);
    // x, y are between 0 and 1
    float x = static_cast<float>(p.x() * size); // p.x() == p[0]
    float y = static_cast<float>(p.y() * size); // p.y() == p[1]
    // use bicubic interpolation

    BicubicMatrix A_inv = make_samples_to_bicubic_coeffs_operator();

    BicubicVector sample_vector = extract_samples(
        static_cast<size_t>(w),
        static_cast<size_t>(h),
        m_image.data(),
        x,
        y,
        m_mode_x,
        m_mode_y);
    BicubicVector bicubic_coeff = A_inv * sample_vector;
    return eval_bicubic_coeffs(bicubic_coeff, x, y);
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
    buffer.reserve(w * h * sizeof(float));

    for (auto i = 0; i < h; i++) {
        for (auto j = 0; j < w; j++) {
            buffer[i * w + j] = m_image(i, j);
        }
    }
    if (path.extension() == ".hdr") {
        auto res = stbi_write_hdr(path.string().c_str(), w, h, 1, buffer.data());
        assert(res);
    }
    if (path.extension() == ".exr") {
        auto res = save_image_exr_red_channel(w, h, buffer, path);
    } else {
        spdlog::trace("[save_image_hdr] format doesn't support \"{}\"", path.string());
        return false;
    }

    spdlog::trace("[save_image_hdr] done \"{}\"", path.string());

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
