#pragma once

#include <Eigen/Core>

#include <array>
#include <cmath>
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>
#include "bicubic_interpolation.h"
namespace wmtk {
class Image
{
    // point coordinates between [0, 1]
    double get(const Eigen::Vector2d& p) const;

    bool save(const std::filesystem::path& path) const;
    void load(const std::filesystem::path& path);

protected:
    Eigen::MatrixXd m_image;
};

double modulo(double x, double n)
{
    double y = fmod(x, n);
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

double Image::get(const Eigen::Vector2d& p) const
{
    double width = m_image.cols();
    double height = m_image.rows();
    auto size = std::max(width, height);
    double x = modulo(p.x() * size, width); // p.x() == p[0]
    double y = modulo(p.y() * size, height); // p.y() == p[1]

    // eval_bicubic_coeffs(m_image, x, y);
}

bool Image::save(const std::filesystem::path& path) const
{
    spdlog::trace("[save_image_png] start \"{}\"", path.string());
    double width = m_image.cols();
    double height = m_image.rows();
    std::vector<uint8_t> buffer;
    buffer.reserve(width * height);

    for (auto i = 0; i < height; i++) {
        for (auto j = 0; j < width; j++) {
            buffer[i * width + j] = double_to_unsignedchar(m_image(i, j));
        }
    }

    auto res = stbi_write_png(
        path.string().c_str(),
        static_cast<int>(width),
        static_cast<int>(height),
        static_cast<int>(1),
        buffer.data(),
        static_cast<int>(width));
    assert(res);

    spdlog::trace("[save_image_png] done \"{}\"", path.string());

    return true;
}

void Image::load(const std::filesystem::path& path)
{
    int width, height, channels;
    unsigned char* buffer = stbi_loadf(path.string(), &width, &height, &channels, 0);
    assert(buffer != nullptr);
    m_image.resize(width, height);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            m_image(i, j) = *buffer[i * width + j];
        }
    }
}
} // namespace wmtk
