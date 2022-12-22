#pragma once

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <filesystem>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "bicubic_interpolation.h"
#include "stb_image_write.h"
namespace wmtk {
class Image
{
protected:
    Eigen::MatrixXd m_image;

public:
    // point coordinates between [0, 1]
    double get(const Eigen::Vector2d& p) const;
    bool set(
        const std::filesystem::path& path,
        const std::function<double(const double&, const double&)>& f,
        const int& width,
        const int& height);
    bool save(const std::filesystem::path& path) const;
    void load(const std::filesystem::path& path);
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
    return m_image((int)x, (int)y);
    // eval_bicubic_coeffs(m_image, x, y);
}

bool Image::save(const std::filesystem::path& path) const
{
    spdlog::trace("[save_image_png] start \"{}\"", path.string());
    double width = m_image.cols();
    double height = m_image.rows();
    std::vector<uint8_t> buffer;
    buffer.reserve(width * height * sizeof(uint8_t));

    for (auto i = 0; i < height; i++) {
        for (auto j = 0; j < width; j++) {
            // wmtk::logger().info("m_image({}, {}) = {}", i, j, m_image(i, j));
            buffer[i * width + j] = m_image(i, j);
            wmtk::logger().info("buffer({}, {}) = {}", i, j, buffer[i * width + j]);
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
// set an image to have same value as the analytical function and save it to the file given
bool Image::set(
    const std::filesystem::path& path,
    const std::function<double(const double&, const double&)>& f,
    const int& width,
    const int& height)
{
    m_image.resize(height, width);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            double u, v;
            u = (double)j / width;
            v = (double)i / height;
            m_image(i, j) = f(u, v);
            wmtk::logger().info("image({}, {}) = {}, = f({}, {})", i, j, f(u, v), u, v);
        }
    }
    auto saved = save(path);
    if (saved)
        return true;
    else
        return false;
}

void Image::load(const std::filesystem::path& path)
{
    int width, height, channels;
    float* buffer = stbi_loadf(path.c_str(), &width, &height, &channels, 0);
    assert(buffer != nullptr);
    m_image.resize(width, height);
    for (int i = 0; i < height; i++) {
        for (int j = 0; j < width; j++) {
            // wmtk::logger().info("buffer [{}, {}] = {}", i, j, buffer[i * width + j]);
            m_image(i, j) = buffer[i * width + j];
        }
    }
}
} // namespace wmtk
