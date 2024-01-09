
#include "sampling_utils.hpp"

namespace wmtk::components::image::utils {
int get_pixel_index_with_image_wrapping_mode(
    const int x,
    const int width,
    const int height,
    const IMAGE_WRAPPING_MODE mode)
{
    auto size = std::max(width, height);
    assert(-size < x && x < 2 * size);
    switch (mode) {
    case IMAGE_WRAPPING_MODE::REPEAT: return (x + size) % size;

    case IMAGE_WRAPPING_MODE::MIRROR_REPEAT:
        if (x < 0)
            return -(x % size);
        else if (x < size)
            return x;
        else
            return size - 1 - (x - size) % size;

    case IMAGE_WRAPPING_MODE::CLAMP_TO_EDGE: return std::clamp(x, 0, size - 1);
    default: return (x + size) % size;
    }
}
uint32_t to_morton_z_order(uint16_t x, uint16_t y)
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

float fetch_texel_offset(const Image& image, int offset)
{
    return image.get_raw_image().data()[offset];
}

float fetch_texel_zorder(const Image& image, int x, int y)
{
    auto zoffset = to_morton_z_order(x, y);
    return fetch_texel_offset(image, zoffset);
}

float fetch_texel_eigen(const Image& image, int x, int y)
{
    return image.get_raw_image()(y, x);
}

float fetch_texel(const Image& image, int x, int y)
{
    return fetch_texel_eigen(image, x, y);
}

bool point_in_triangle(
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

Classification point_in_triangle_quick(
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

Classification pixel_inside_triangle(
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

} // namespace wmtk::components::image::utils
