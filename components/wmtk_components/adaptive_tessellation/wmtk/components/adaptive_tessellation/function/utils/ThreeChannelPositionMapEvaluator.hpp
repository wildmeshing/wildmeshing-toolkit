#pragma once
#include <memory>
#include <wmtk/Types.hpp>
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>
#include <wmtk/components/adaptive_tessellation/image/utils/SamplingParameters.hpp>
#include <wmtk/components/adaptive_tessellation/image/utils/sampling_utils.hpp>

#include <wmtk/utils/Logger.hpp>

#include <cmath>
namespace image = wmtk::components::image;
namespace wmtk::components {
namespace function::utils {

class ThreeChannelPositionMapEvaluator
{
protected:
    const std::array<std::shared_ptr<image::Image>, 3> m_images;
    const std::array<std::shared_ptr<image::Sampling>, 3> m_analytical_funcs;
    image::SAMPLING_METHOD m_sampling_method = image::SAMPLING_METHOD::Bicubic;
    image::IMAGE_WRAPPING_MODE m_wrapping_mode = image::IMAGE_WRAPPING_MODE::MIRROR_REPEAT;

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
        const std::array<std::shared_ptr<image::Sampling>, 3> funcs,
        const image::SAMPLING_METHOD sampling_method = image::SAMPLING_METHOD::Analytical);

    int width() const;
    int height() const;
    image::IMAGE_WRAPPING_MODE get_wrapping_mode() const;
    image::SAMPLING_METHOD get_sampling_method() const;
    void set_sampling_method(const image::SAMPLING_METHOD method);
    void set_wrapping_mode(const image::IMAGE_WRAPPING_MODE mode);

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
                res[k] = m_analytical_funcs[k]->sample(uv);
            }
            return res;
        } else {
            throw std::runtime_error("uv_to_position with three channel images has to be using "
                                     "Bicubic/Nearest/Bilinear as the sampling method.");
        }
    }

    template <typename T>
    Vector3<T> pixel_coord_to_position_bilinear(const Vector2<T>& uv) const
    {
        return image::utils::sample_bilinear_with_pixel_coord(m_images, uv.x(), uv.y());
    }

    template <typename T>
    Vector3<T> pixel_index_to_position(int x, int y) const
    {
        return image::utils::fetch_texel_eigen(m_images, x, y);
    }

    std::pair<int, int> pixel_index(const Vector2<double>& uv) const
    {
        int w = m_images[0]->width();
        int h = m_images[0]->height();
        const int sx = std::clamp(static_cast<int>(std::round(uv.x() * w)), 0, w - 1);
        const int sy = std::clamp(static_cast<int>(std::round(uv.y() * h)), 0, h - 1);
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

    int pixel_index_floor(const double xcoord) const
    {
        int w = m_images[0]->width();
        int h = m_images[0]->height();
        int size = std::min(w, h);
        auto coord = xcoord * size;
        int floor = 0;
        if (coord <= 0.5) {
            floor = 0;

        } else if (coord + 0.5 >= static_cast<double>(size)) {
            floor = size - 1;

        } else {
            assert(1 < size);
            int a = size - 2;
            int b = static_cast<size_t>(coord - 0.5);
            floor = std::min(a, b);
        }

        return floor;
    }

    std::pair<int, int> pixel_index_floor(const Vector2<double>& uv) const
    {
        return {pixel_index_floor(uv.x()), pixel_index_floor(uv.y())};
    }

    int pixel_index_ceil(const double xcoord) const
    {
        int w = m_images[0]->width();
        int h = m_images[0]->height();
        int size = std::min(w, h);
        int ceil = 0;
        auto coord = xcoord * size;
        if (coord <= 0.5f) {
            ceil = 0;
        } else if (coord + 0.5f >= static_cast<double>(size)) {
            ceil = size - 1;

        } else {
            assert(1 < size);
            int a = size - 2;
            int b = static_cast<size_t>(coord - 0.5);
            ceil = std::min(a, b) + 1;
        }

        return ceil;
    }

    std::pair<int, int> pixel_index_ceil(const Vector2<double>& uv) const
    {
        return {pixel_index_ceil(uv.x()), pixel_index_ceil(uv.y())};
    }

    float pixel_size() const { return m_images[0]->pixel_size(); }
    int image_size() const { return m_images[0]->size(); }
};

} // namespace function::utils
} // namespace wmtk::components
