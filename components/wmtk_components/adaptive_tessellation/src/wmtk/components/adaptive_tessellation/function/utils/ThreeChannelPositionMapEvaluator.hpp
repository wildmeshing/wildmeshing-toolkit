#pragma once
#include <memory>
#include <wmtk/Types.hpp>
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>
#include <wmtk/components/adaptive_tessellation/image/utils/SamplingParameters.hpp>
#include <wmtk/components/adaptive_tessellation/image/utils/sampling_utils.hpp>

namespace image = wmtk::components::adaptive_tessellation::image;
namespace wmtk::components::adaptive_tessellation {
namespace function::utils {

class ThreeChannelPositionMapEvaluator
{
protected:
    const std::array<std::shared_ptr<image::Image>, 3> m_images;
    const std::array<std::shared_ptr<image::SamplingAnalyticFunction>, 3> m_analytical_funcs;
    const image::SAMPLING_METHOD m_sampling_method;
    const image::IMAGE_WRAPPING_MODE m_wrapping_mode = image::IMAGE_WRAPPING_MODE::MIRROR_REPEAT;

public:
    ~ThreeChannelPositionMapEvaluator();
    ThreeChannelPositionMapEvaluator(ThreeChannelPositionMapEvaluator& other);

    /**
     * @brief Construct a position map evaluator from a three channel image
     *
     * @param
     */
    ThreeChannelPositionMapEvaluator(
        const std::array<std::shared_ptr<image::Image>, 3> images,
        const image::SAMPLING_METHOD sampling_method = image::SAMPLING_METHOD::Bicubic,
        const image::IMAGE_WRAPPING_MODE wrapping_mode = image::IMAGE_WRAPPING_MODE::MIRROR_REPEAT);

    ThreeChannelPositionMapEvaluator(
        const std::array<std::shared_ptr<image::SamplingAnalyticFunction>, 3> funcs,
        const image::SAMPLING_METHOD sampling_method = image::SAMPLING_METHOD::Analytical);

    int width() const;
    int height() const;
    image::IMAGE_WRAPPING_MODE get_wrapping_mode() const;
    image::SAMPLING_METHOD get_sampling_method() const { return m_sampling_method; }

    template <typename T>
    Vector3<T> uv_to_position(const Vector2<T>& uv) const
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
    std::pair<int, int> pixel_index(const Vector2<T>& uv) const
    {
        auto [xx, yy] =
            m_images[0]->get_pixel_index(image::get_double(uv.x()), image::get_double(uv.y()));
        return {
            get_pixel_index_with_image_wrapping_mode(xx, width(), height(), get_wrapping_mode()),
            get_pixel_index_with_image_wrapping_mode(yy, width(), height(), get_wrapping_mode())};
    }
};

} // namespace function::utils
} // namespace wmtk::components::adaptive_tessellation
