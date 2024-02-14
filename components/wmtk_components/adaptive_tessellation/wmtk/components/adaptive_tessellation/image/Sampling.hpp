#pragma once
#include <Eigen/Core>
#include <wmtk/Types.hpp>
#include "Image.hpp"
#include "utils/SamplingParameters.hpp"
#include "utils/sampling_utils.hpp"
namespace wmtk::components::image {

class Sampling
{
public:
    virtual ~Sampling(){};

public:
    virtual double sample(const Vector2<double> uv) const = 0;
    virtual utils::DScalar sample(const Vector2<utils::DScalar>& uv) const = 0;
};


enum SamplingAnalyticFunction_FunctionType { Linear, Quadratic, Periodic, Gaussian };
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
    template <typename S>
    auto evaluate_gaussian(const S& u, const S& v) const
    {
        return C * exp(-(pow(u - A, 2) + pow(v - B, 2)) / (2 * 0.1 * 0.1));
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
    utils::DScalar sample(const Vector2<utils::DScalar>& uv) const override
    {
        return evaluate<utils::DScalar>(uv.x(), uv.y());
    }

    template <typename S>
    S evaluate(const S& u, const S& v) const
    {
        if (m_type == Linear) {
            return evaluate_linear<S>(u, v);
        } else if (m_type == Periodic) {
            return evaluate_periodic<S>(u, v);
        } else if (m_type == Gaussian) {
            return evaluate_gaussian<S>(u, v);
        } else
            return static_cast<S>(0.0);
    }
};

enum ProceduralFunctionType { Terrain };
class ProceduralFunction : public Sampling
{
protected:
    ProceduralFunctionType m_type = ProceduralFunctionType::Terrain;

protected:
    template <typename S>
    auto noise(const S& u, const S& v) const
    {
        return .4 * (5.5 * sin(u) + 8. * cos(v));
    }
    template <typename S>
    auto evaluate_procedural_terrain(const S& u, const S& v) const
    {
        // this function generates the terrain height
        S value = (S)0.;
        double amplitude = 1.;
        double freq = 1.;

        for (int i = 0; i < 8; i++) {
            // From Dave_Hoskins https://www.shadertoy.com/user/Dave_Hoskins
            value += .25 - pow(noise(40 * u * freq, 40 * v * freq) - .3, 2) * amplitude + 2.5;

            amplitude *= .37;

            freq *= 2.05;
        }

        return .008 * (value * 1.5 - 1.0);
    }
    template <typename S>
    S evaluate(const S& u, const S& v) const
    {
        if (m_type == Terrain) {
            return evaluate_procedural_terrain(u, v);
        } else
            return static_cast<S>(0.0);
    }

public:
    // make a contructor
    ProceduralFunction(const ProceduralFunctionType type)
        : m_type(type){};

    double sample(const Vector2<double> uv) const override
    {
        return evaluate<double>(uv.x(), uv.y());
    }
    utils::DScalar sample(const Vector2<utils::DScalar>& uv) const override
    {
        return evaluate<utils::DScalar>(uv.x(), uv.y());
    }
    // set an image to have same value as the analytical function and save it to the file given
    bool convert_to_exr(const int w, const int h)
    {
        Image img(h, w);

        for (int i = 0; i < h; i++) {
            for (int j = 0; j < w; j++) {
                double u, v;
                u = (static_cast<double>(j) + 0.5) / static_cast<double>(w);
                v = (static_cast<double>(i) + 0.5) / static_cast<double>(h);
                img.set(i, j, evaluate<double>(u, v));
            }
        }

        img.save("terrain.exr");
        return true;
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
    utils::DScalar sample(const Vector2<utils::DScalar>& uv) const override
    {
        return sample_T<utils::DScalar>(uv);
    }
};

} // namespace wmtk::components::image