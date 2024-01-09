#include "ThreeChannelPositionMapEvaluator.hpp"
namespace image = wmtk::components::image;
namespace wmtk::components {
namespace function::utils {
ThreeChannelPositionMapEvaluator::~ThreeChannelPositionMapEvaluator() = default;


ThreeChannelPositionMapEvaluator::ThreeChannelPositionMapEvaluator(
    ThreeChannelPositionMapEvaluator& other)
    : m_images(other.m_images)
    , m_sampling_method(other.m_sampling_method)
    , m_wrapping_mode(other.m_wrapping_mode)
{}
ThreeChannelPositionMapEvaluator::ThreeChannelPositionMapEvaluator(
    const std::array<std::shared_ptr<image::SamplingAnalyticFunction>, 3> funcs,
    const image::SAMPLING_METHOD sampling_method)
    : m_analytical_funcs(funcs)
    , m_sampling_method(sampling_method)
{
    assert(sampling_method == image::SAMPLING_METHOD::Analytical);
}

ThreeChannelPositionMapEvaluator::ThreeChannelPositionMapEvaluator(
    const std::array<std::shared_ptr<image::Image>, 3> images,
    const image::SAMPLING_METHOD sampling_method,
    const image::IMAGE_WRAPPING_MODE wrapping_mode)
    : m_images(images)
    , m_sampling_method(sampling_method)
    , m_wrapping_mode(wrapping_mode)
{ // all the 3 channel images should have same size
    assert(m_images[0]->width() == m_images[1]->width());
    assert(m_images[0]->width() == m_images[2]->width());
    assert(m_images[0]->height() == m_images[1]->height());
    assert(m_images[0]->height() == m_images[2]->height());
}
int ThreeChannelPositionMapEvaluator::width() const
{
    if (m_sampling_method == image::SAMPLING_METHOD::Analytical) {
        throw std::runtime_error(
            "ThreeChannelPositionMapEvaluator::width() is not defined for analytical functions.");
    }
    return m_images[0]->width();
}
int ThreeChannelPositionMapEvaluator::height() const
{
    if (m_sampling_method == image::SAMPLING_METHOD::Analytical) {
        throw std::runtime_error(
            "ThreeChannelPositionMapEvaluator::height() is not defined for analytical functions.");
    }
    return m_images[0]->height();
}
image::IMAGE_WRAPPING_MODE ThreeChannelPositionMapEvaluator::get_wrapping_mode() const
{
    if (m_sampling_method == image::SAMPLING_METHOD::Analytical) {
        throw std::runtime_error("ThreeChannelPositionMapEvaluator::get_wrapped_mode() is not "
                                 "defined for analytical functions.");
    }
    return m_wrapping_mode;
}


} // namespace function::utils
} // namespace wmtk::components