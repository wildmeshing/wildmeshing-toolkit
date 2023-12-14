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

} // namespace wmtk::function::utils