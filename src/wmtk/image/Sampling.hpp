#pragma once
#include <Eigen/Core>
#include "Image.hpp"
#include "bicubic_interpolation.hpp"

namespace wmtk {
namespace image {
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

template <typename SamplingType>
class SamplingAnalyticFunction : public Sampling
{
protected:
    std::function<SamplingType(SamplingType, SamplingType)> m_f;
};

// Specialization for SamplingAnalyticFunction<double>
template <>
class SamplingAnalyticFunction<double> : public Sampling
{
protected:
    std::function<double(double, double)> m_f;

public:
    SamplingAnalyticFunction(std::function<double(double, double)> f)
        : m_f(f)
    {}

    double sample(const double u, const double v) const override { return m_f(u, v); }

    DScalar sample(const DScalar& u, const DScalar& v) const override
    {
        // Handle the case where SamplingType is double
        //  return a default DScalar
        return DScalar{m_f(u.getValue(), v.getValue()),
                       Eigen::Vector2d::Zero(),
                       Eigen::Matrix2d::Zero()};
    }
};

// Specialization for SamplingAnalyticFunction<DScalar>
template <>
class SamplingAnalyticFunction<Sampling::DScalar> : public Sampling
{
protected:
    std::function<Sampling::DScalar(Sampling::DScalar, Sampling::DScalar)> m_f;

public:
    SamplingAnalyticFunction(
        std::function<Sampling::DScalar(Sampling::DScalar, Sampling::DScalar)> f)
        : m_f(f)
    {}

    double sample(const double u, const double v) const override
    {
        // Handle the case where SamplingType is DScalar
        //  return a default double
        return m_f(DScalar(u), DScalar(v)).getValue();
    }

    Sampling::DScalar sample(const Sampling::DScalar& u, const Sampling::DScalar& v) const override
    {
        return m_f(u, v);
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
} // namespace image
} // namespace wmtk
