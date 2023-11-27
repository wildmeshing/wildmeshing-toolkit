#pragma once
#include <Eigen/Core>
#include "Image.hpp"
#include "bicubic_interpolation.hpp"

namespace wmtk::components::adaptive_tessellation::image {
enum class SAMPLING_MODE { BICUBIC, SPLINE };
class Sampling
{
public:
    using DScalar = DScalar2<double, Eigen::Matrix<double, -1, 1>, Eigen::Matrix<double, -1, -1>>;
    virtual ~Sampling(){};

public:
    virtual double sample(const double u, const double v) const = 0;
    virtual DScalar sample(const DScalar& u, const DScalar& v) const = 0;
};


enum SamplingAnalyticFunction_FunctionType { Linear, Quadratic };
class SamplingAnalyticFunction : public Sampling
{
public:
    using FunctionType = SamplingAnalyticFunction_FunctionType;

protected:
    FunctionType m_type = FunctionType::Linear;
    double A = 0.0;
    double B = 0.0;
    double C = 0.0;

    template <typename S>
    auto evaluate(const S& u, const S& v) const
    {
        if (m_type == Linear) {
            return evaluate_linear(u, v);
        } else
            return static_cast<S>(0.0);
    }

    template <typename S>
    auto evaluate_linear(const S& u, const S& v) const
    {
        return A * u + B * v + C;
    }

public:
    // make a contructor
    SamplingAnalyticFunction(
        const FunctionType type,
        const double a,
        const double b,
        const double c)
        : m_type(type)
        , A(a)
        , B(b)
        , C(c)
    {}


    void set_coefficients(double a, const double b, const double c)
    {
        A = a;
        B = b;
        C = c;
    }
    double sample(const double u, const double v) const override { return evaluate<double>(u, v); }
    DScalar sample(const DScalar& u, const DScalar& v) const override
    {
        return evaluate<DScalar>(u, v);
    }
};

template <typename Derived>
class SamplingImage : public Sampling
{
protected:
    const Image& m_image;

public:
    SamplingImage(const Image& img)
        : m_image(img)
    {
        assert(m_image.width() == m_image.height());
        assert(m_image.width() != 0);
    }

public:
    double sample(const double u, const double v) const override
    {
        return static_cast<const Derived*>(this)->sample(u, v);
    }
    DScalar sample(const DScalar& u, const DScalar& v) const override
    {
        return static_cast<const Derived*>(this)->sample(u, v);
    }
};

class SamplingBicubic : public SamplingImage<SamplingBicubic>
{
public:
    using Super = SamplingImage<SamplingBicubic>;
    using Super::Super;
    template <class T>
    T sample_T(T u, T v) const
    {
        auto w = m_image.width();
        auto h = m_image.height();
        // x, y are between 0 and 1
        T x = u * static_cast<std::decay_t<T>>(w);
        T y = v * static_cast<std::decay_t<T>>(h);

        // use bicubic interpolation
        BicubicVector<float> sample_vector = extract_samples(
            static_cast<size_t>(w),
            static_cast<size_t>(h),
            m_image.get_raw_image().data(),
            wmtk::image::get_value(x),
            wmtk::image::get_value(y),
            m_image.get_wrapping_mode_x(),
            m_image.get_wrapping_mode_y());
        BicubicVector<float> bicubic_coeff = get_bicubic_matrix() * sample_vector;
        return eval_bicubic_coeffs(bicubic_coeff, x, y);
    }
    double sample(const double u, const double v) const override { return sample_T<double>(u, v); }
    DScalar sample(const DScalar& u, const DScalar& v) const override
    {
        return sample_T<DScalar>(u, v);
    }
};
} // namespace wmtk::components::adaptive_tessellation::image
