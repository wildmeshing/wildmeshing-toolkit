#pragma once
#include <memory>
#include <wmtk/Types.hpp>
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/components/adaptive_tessellation/image/utils/SamplingParameters.hpp>
#include <wmtk/components/adaptive_tessellation/image/utils/sampling_utils.hpp>

namespace image = wmtk::components::adaptive_tessellation::image;
namespace wmtk::function::utils {

class ThreeChannelPositionMapEvaluator
{
protected:
    const std::array<image::Image, 3>& m_images;
    const image::SAMPLING_METHOD m_sampling_method;
    const image::IMAGE_WRAPPING_MODE m_wrapping_mode;

public:
    ~ThreeChannelPositionMapEvaluator();
    ThreeChannelPositionMapEvaluator(
        ThreeChannelPositionMapEvaluator&&); // move assignment operator

    /**
     * @brief Construct a position map evaluator from a three channel image
     *
     * @param
     */
    ThreeChannelPositionMapEvaluator(
        const std::array<image::Image, 3>& images,
        const image::SAMPLING_METHOD sampling_method,
        const image::IMAGE_WRAPPING_MODE wrapping_mode = image::IMAGE_WRAPPING_MODE::REPEAT);

    int width() const { return m_images[0].width(); }
    int height() const { return m_images[0].height(); }
    image::IMAGE_WRAPPING_MODE get_wrapping_mode() const { return m_wrapping_mode; }

    template <typename T>
    Vector3<T> uv_to_position(const Vector2<T>& uv) const;

    template <typename T>
    std::pair<int, int> pixel_index(const Vector2<T>& uv) const;
};

} // namespace wmtk::function::utils
