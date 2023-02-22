#pragma once

//#include "Image.h"
#include "Displacement.h"
#include "Image.h"
#include "bicubic_interpolation.h"

#include <Eigen/Core>


namespace wmtk {

class DisplacementBicubic : public Displacement
{
    using Scalar = double;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        img_; // displacement as pixel image

    int img_size_ = 0;
    double inv_img_size_ = 0;

    ::WrappingMode wrap_x_ = ::WrappingMode::CLAMP_TO_EDGE;
    ::WrappingMode wrap_y_ = ::WrappingMode::CLAMP_TO_EDGE;

public:
    DisplacementBicubic(
        const Image& img,
        const ::WrappingMode& wrap_x = ::WrappingMode::CLAMP_TO_EDGE,
        const ::WrappingMode& wrap_y = ::WrappingMode::CLAMP_TO_EDGE)
        : img_size_(img.width())
        , inv_img_size_(1.0 / img.width())
        , wrap_x_(wrap_x)
        , wrap_y_(wrap_y)
        , img_(img.get_raw_image())
    {
        assert(img_.rows() == img_.cols());
        assert(img_.rows() != 0);
    }

    template <class T>
    std::decay_t<T> get(const T& u, const T& v) const
    {
        // x, y are between 0 and 1
        auto x = u * static_cast<std::decay_t<T>>(img_size_);
        auto y = v * static_cast<std::decay_t<T>>(img_size_);

        // use bicubic interpolation
        BicubicVector<float> sample_vector = extract_samples(
            static_cast<size_t>(img_size_),
            static_cast<size_t>(img_size_),
            img_.data(),
            wmtk::get_value(x),
            wmtk::get_value(y),
            wrap_x_,
            wrap_y_);
        BicubicVector<float> bicubic_coeff = get_bicubic_matrix() * sample_vector;
        return eval_bicubic_coeffs(bicubic_coeff, x, y);
    }

private:
    Scalar px_to_param(const Scalar& px_x) { return (px_x + 0.5) * inv_img_size_; }
    Scalar param_to_px(const Scalar& u) { return u * img_size_ - 0.5; }
};
} // namespace wmtk
