#pragma once
#include <memory>
#include <wmtk/Types.hpp>
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>
#include <wmtk/components/adaptive_tessellation/image/utils/SamplingParameters.hpp>
#include <wmtk/components/adaptive_tessellation/image/utils/sampling_utils.hpp>

namespace image = wmtk::components::image;
namespace wmtk::components {
namespace function::utils {

class ThreeChannelPositionMapEvaluator
{
protected:
    const std::array<std::shared_ptr<image::Image>, 3> m_images;
    const std::array<std::shared_ptr<image::SamplingAnalyticFunction>, 3> m_analytical_funcs;
    const image::SAMPLING_METHOD m_sampling_method = image::SAMPLING_METHOD::Bicubic;
    const image::IMAGE_WRAPPING_MODE m_wrapping_mode = image::IMAGE_WRAPPING_MODE::MIRROR_REPEAT;

public:
    ThreeChannelPositionMapEvaluator();
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
            return image::utils::sample_bicubic(m_images, uv.x(), uv.y());
        } else if (m_sampling_method == image::SAMPLING_METHOD::Nearest) {
            return image::utils::sample_nearest(m_images, uv.x(), uv.y());
        } else if (m_sampling_method == image::SAMPLING_METHOD::Bilinear) {
            return image::utils::sample_bilinear(m_images, uv.x(), uv.y());
        } else if (m_sampling_method == image::SAMPLING_METHOD::Analytical) {
            Eigen::Vector<T, 3> res;
            for (size_t k = 0; k < 3; ++k) {
                res[k] = m_analytical_funcs[k]->evaluate(uv.x(), uv.y());
            }
            return res;
        } else {
            throw std::runtime_error("uv_to_position with three channel images has to be using "
                                     "Bicubic/Nearest/Bilinear as the sampling method.");
        }
    }


    std::pair<int, int> pixel_index(const Vector2<double>& uv) const
    {
        int w = m_images[0]->width();
        int h = m_images[0]->height();

        const auto sx = std::clamp(static_cast<int>(uv.x()), 0, w - 1);
        const auto sy = std::clamp(static_cast<int>(uv.y()), 0, h - 1);
        return {sx, sy};
        // auto [xx, yy] = m_images[0]->get_pixel_index(uv.x(), uv.y());
        // return {
        //     image::utils::get_pixel_index_with_image_wrapping_mode(
        //         xx,
        //         width(),
        //         height(),
        //         get_wrapping_mode()),
        //     image::utils::get_pixel_index_with_image_wrapping_mode(
        //         yy,
        //         width(),
        //         height(),
        //         get_wrapping_mode())};
    }
};

} // namespace function::utils
} // namespace wmtk::components
