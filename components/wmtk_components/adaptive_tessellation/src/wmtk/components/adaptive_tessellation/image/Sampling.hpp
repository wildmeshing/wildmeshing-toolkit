#pragma once
#include <Eigen/Core>
#include <wmtk/Types.hpp>
#include "Image.hpp"
#include "utils/SamplingParameters.hpp"
#include "utils/sampling_utils.hpp"
namespace wmtk::components::adaptive_tessellation::image {

class Sampling
{
public:
    virtual ~Sampling(){};

public:
    virtual double sample(const Vector2<double> uv) const = 0;
    virtual DScalar sample(const Vector2<DScalar>& uv) const = 0;
};


enum SamplingAnalyticFunction_FunctionType { Linear, Quadratic, Periodic };
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
    auto evaluate_linear(const S& u, const S& v) const
    {
        return A * u + B * v + C;
    }
    template <typename S>
    auto evaluate_periodic(const S& u, const S& v) const
    {
        return C * sin(A * M_PI * u) * cos(B * M_PI * v);
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
    double sample(const Vector2<double> uv) const override
    {
        return evaluate<double>(uv.x(), uv.y());
    }
    DScalar sample(const Vector2<DScalar>& uv) const override
    {
        return evaluate<DScalar>(uv.x(), uv.y());
    }

    template <typename S>
    S evaluate(const S& u, const S& v) const
    {
        if (m_type == Linear) {
            return evaluate_linear<S>(u, v);
        } else if (m_type == Periodic) {
            return evaluate_periodic<S>(u, v);
        } else
            return static_cast<S>(0.0);
    }
};


class SamplingImage : public Sampling
{
protected:
    const Image& m_image;
    const IMAGE_WRAPPING_MODE m_mode;
    const SAMPLING_METHOD m_sampling_method;

public:
    SamplingImage(
        const Image& img,
        const IMAGE_WRAPPING_MODE mode = IMAGE_WRAPPING_MODE::REPEAT,
        const SAMPLING_METHOD sampling_method = SAMPLING_METHOD::Bicubic)
        : m_image(img)
        , m_mode(mode)
        , m_sampling_method(sampling_method)
    {
        assert(m_image.width() == m_image.height());
        assert(m_image.width() != 0);
    }

public:
    IMAGE_WRAPPING_MODE get_wrapping_mode() const { return m_mode; }
    SAMPLING_METHOD get_sampling_method() const { return m_sampling_method; }
    template <typename T>
    T sample_T(const Vector2<T> uv) const
    {
        if (get_sampling_method() == SAMPLING_METHOD::Bicubic) {
            return utils::sample_bicubic<T>(m_image, uv.x(), uv.y());
        } else if (get_sampling_method() == SAMPLING_METHOD::Nearest) {
            return utils::sample_nearest<T>(m_image, uv.x(), uv.y());
        } else if (get_sampling_method() == SAMPLING_METHOD::Bilinear) {
            return utils::sample_bilinear<T>(m_image, uv.x(), uv.y());
        } else {
            throw std::runtime_error("Sampling method has to be Bicubic/Nearest/Bilinear.");
        }
    }
    double sample(const Vector2<double> uv) const override { return sample_T<double>(uv); }
    DScalar sample(const Vector2<DScalar>& uv) const override { return sample_T<DScalar>(uv); }
};

} // namespace wmtk::components::adaptive_tessellation::image