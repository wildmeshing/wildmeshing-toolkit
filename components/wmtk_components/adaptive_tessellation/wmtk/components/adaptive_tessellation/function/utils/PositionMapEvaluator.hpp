#pragma once
#include <memory>
#include <wmtk/Types.hpp>
#include <wmtk/components/adaptive_tessellation/image/Image.hpp>
#include <wmtk/components/adaptive_tessellation/image/Sampling.hpp>

namespace wmtk::components::image {
class Image;
class SamplingAnalyticFunction;
} // namespace wmtk::components::image
namespace image = wmtk::components::image;
namespace wmtk::function::utils {
class PositionMapEvaluator
{
protected:
    std::unique_ptr<image::Sampling> m_sampling;

public:
    PositionMapEvaluator();
    ~PositionMapEvaluator();
    PositionMapEvaluator(PositionMapEvaluator&&); // move assignment operator
    PositionMapEvaluator& operator=(PositionMapEvaluator&&); // move assignment operator

    /**
     * @brief Construct a new Dofs To Position object using a displacement map (requires a
     * sampler)
     *
     * @param image
     */
    PositionMapEvaluator(const image::Image& image);

    PositionMapEvaluator(
        const image::SamplingAnalyticFunction::FunctionType type,
        const double a,
        const double b,
        const double c);


    // Dont forget to update this if we change autodiff tyeps (add declarations in the cpp)
    template <typename T>
    Vector3<T> uv_to_pos(const Vector2<T>& uv) const
    {
        return Vector3<T>(uv.x(), uv.y(), m_sampling->sample(uv));
    }

    // Dont forget to update this if we change autodiff tyeps (add declarations in the cpp)
    template <typename T>
    T uv_to_height(const Vector2<T>& uv) const
    {
        return m_sampling->sample(uv);
    }
};

} // namespace wmtk::function::utils
