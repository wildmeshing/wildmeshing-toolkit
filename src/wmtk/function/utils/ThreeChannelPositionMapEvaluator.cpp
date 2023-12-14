#include "ThreeChannelPositionMapEvaluator.hpp"
namespace image = wmtk::components::adaptive_tessellation::image;
namespace wmtk::function::utils {

ThreeChannelPositionMapEvaluator::~ThreeChannelPositionMapEvaluator() = default;

ThreeChannelPositionMapEvaluator::ThreeChannelPositionMapEvaluator(
    ThreeChannelPositionMapEvaluator&&) = default; // move assignment operator

ThreeChannelPositionMapEvaluator::ThreeChannelPositionMapEvaluator(
    const std::array<image::Image, 3>& images,
    const image::SAMPLING_METHOD sampling_method,
    const image::IMAGE_WRAPPING_MODE wrapping_mode)
    : m_images(images)
    , m_sampling_method(sampling_method)
    , m_wrapping_mode(wrapping_mode)
{ // all the 3 channel images should have same size
    assert(m_images[0].width() == m_images[1].width());
    assert(m_images[0].width() == m_images[2].width());
    assert(m_images[0].height() == m_images[1].height());
    assert(m_images[0].height() == m_images[2].height());
}

template <typename T>
Vector3<T> ThreeChannelPositionMapEvaluator::uv_to_position(const Vector2<T>& uv) const
{
    if (m_sampling_method == image::SAMPLING_METHOD::Bicubic) {
        return image::sample_bicubic(m_images, uv.x(), uv.y());
    } else if (m_sampling_method == image::SAMPLING_METHOD::Nearest) {
        return image::sample_nearest(m_images, uv.x(), uv.y());
    } else if (m_sampling_method == image::SAMPLING_METHOD::Bilinear) {
        return image::sample_bilinear(m_images, uv.x(), uv.y());
    } else {
        throw std::runtime_error("uv_to_position with three channel images has to be using "
                                 "Bicubic/Nearest/Bilinear as the sampling method.");
    }
}

template <typename T>
std::pair<int, int> ThreeChannelPositionMapEvaluator::pixel_index(const Vector2<T>& uv) const
{
    auto [xx, yy] =
        m_images[0].get_pixel_index(image::get_double(uv.x()), image::get_double(uv.y()));
    return {
        get_pixel_index_with_image_wrapping_mode(xx, width(), height(), get_wrapping_mode()),
        get_pixel_index_with_image_wrapping_mode(yy, width(), height(), get_wrapping_mode())};
}

} // namespace wmtk::function::utils