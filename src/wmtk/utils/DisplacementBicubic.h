#pragma once

//#include "Image.h"
#include <wmtk/utils/LineQuadrature.hpp>
#include "Displacement.h"
#include "Image.h"
#include "bicubic_interpolation.h"

#include <Eigen/Core>


namespace wmtk {
// Derived displacement class
struct DisplacementBicubic : public DisplacementImage<DisplacementBicubic>
{
    // Reuse parent constructor
    using DisplacementImage<DisplacementBicubic>::DisplacementImage;

    // Typedef parent class
    // using Super = DisplacementImage<DisplacementBicubic>;
    // Repeated constructor
    // DisplacementBicubic(const Image &img) : Super(img) {}

    // Templated getter using autodiff
    template <typename T>
    T get(T u, T v) const
    {
        auto w = m_image.width();
        auto h = m_image.height();
        // x, y are between 0 and 1
        auto x = u * static_cast<std::decay_t<T>>(w);
        auto y = v * static_cast<std::decay_t<T>>(h);

        // use bicubic interpolation
        BicubicVector<float> sample_vector = extract_samples(
            static_cast<size_t>(w),
            static_cast<size_t>(h),
            m_image.get_raw_image().data(),
            wmtk::get_value(x),
            wmtk::get_value(y),
            m_image.get_wrapping_mode_x(),
            m_image.get_wrapping_mode_y());
        BicubicVector<float> bicubic_coeff = get_bicubic_matrix() * sample_vector;
        return eval_bicubic_coeffs(bicubic_coeff, x, y);
        return x;
    }
};
} // namespace wmtk
