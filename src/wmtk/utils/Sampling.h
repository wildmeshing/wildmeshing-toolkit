#pragma once
#include <nanospline/BSplinePatch.h>
#include <nanospline/arc_length.h>
#include <Eigen/Core>
#include "Image.h"
#include "bicubic_interpolation.h"

namespace wmtk {
enum class SAMPLING_MODE { BICUBIC, SPLINE };
class Sampling
{
public:
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;
    virtual ~Sampling(){};

public:
    virtual double sample(const double u, const double v) const = 0;
    virtual DScalar sample(const DScalar& u, const DScalar& v) const = 0;
};

template <typename Derived>
class SamplingImage : public Sampling
{
protected:
    const wmtk::Image& m_image;

public:
    SamplingImage(const wmtk::Image& img)
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
    }
    double sample(double u, double v) const { return sample_T<double>(u, v); }
    DScalar sample(DScalar u, DScalar v) const { return sample_T<DScalar>(u, v); }
};

class SamplingSpline : public SamplingImage<SamplingSpline>
{
    using Scalar = double;
    // Typedef parent class
    using Super = SamplingImage<SamplingSpline>;
    static constexpr int m_SPLINE_DEGREE = 3;

    int m_img_size = 0;
    double m_inv_img_size = 0;

    int m_n_grid_points = 0;
    int m_n_control_points = 0;
    int m_pxl_offset = ((m_SPLINE_DEGREE - 1) / 2);
    double m_inv_n_grid_points = 0;

    Eigen::MatrixX<Scalar> m_control_mat;

    nanospline::BSplinePatch<Scalar, 3, m_SPLINE_DEGREE, m_SPLINE_DEGREE> m_patch;

public:
    // Repeated constructor
    SamplingSpline(const Image& img)
        : Super(img)
        , m_img_size(img.width())
        , m_inv_img_size(1.0 / img.width())
    {
        assert(m_image.width() == m_image.height());
        assert(m_image.width() != 0);

        ::WrappingMode wrap_x = img.get_wrapping_mode_x();
        ::WrappingMode wrap_y = img.get_wrapping_mode_y();
        auto img_raw = img.get_raw_image();

        m_n_grid_points = m_img_size;
        m_inv_n_grid_points = m_inv_img_size;
        m_n_control_points = m_n_grid_points + m_SPLINE_DEGREE - 1;

        // create knot vector
        const int n_knots = m_n_control_points + m_SPLINE_DEGREE + 1;
        Eigen::VectorX<Scalar> knots;
        knots.resize(n_knots);
        for (int i = 0; i < n_knots; ++i) {
            knots[i] = i - m_SPLINE_DEGREE;
        }

        // create control matrix
        m_control_mat.resize(m_n_control_points * m_n_control_points, 3);
        for (int i = 0; i < m_n_control_points; ++i) {
            const int i_px = cp_to_px(i);
            const int img_i_px = img.get_coordinate(i_px, wrap_x);
            for (int j = 0; j < m_n_control_points; ++j) {
                const int j_px = cp_to_px(j);
                const int img_j_px = img.get_coordinate(j_px, wrap_y);
                m_control_mat.row(j * m_n_control_points + i) =
                    Eigen::Matrix<Scalar, 1, 3>(i_px, j_px, img_raw(img_i_px, img_j_px));
            }
        }

        m_patch.set_control_grid(m_control_mat);

        m_patch.set_knots_u(knots);
        m_patch.set_knots_v(knots);
        m_patch.initialize();
    }

    // Transformation functions
    int cp_to_px(const int& i_cp) const { return i_cp - m_pxl_offset; }
    double px_to_param(const double& px_x) const { return (px_x + 0.5) * m_inv_n_grid_points; }
    double param_to_px(const double& u) const { return u * m_n_grid_points - 0.5; }
    double cp_to_param(const int& i_cp) const { return px_to_param(cp_to_px(i_cp)); }

    // Assume input function is defined on the unit square: u,v in [0,1].
    void set(const std::function<Scalar(const Scalar&, const Scalar&)>& f)
    {
        for (int i_cp = 0; i_cp < m_n_control_points; ++i_cp) {
            const double u = cp_to_param(i_cp);
            for (int j_cp = 0; j_cp < m_n_control_points; ++j_cp) {
                const double v = cp_to_param(j_cp);
                const double w = f(u, v);
                m_control_mat.row(i_cp * m_n_control_points + j_cp)[2] = w;
            }
        }
    }

    template <class T>
    T sample(const T& u, const T& v) const
    {
        const double px_x = param_to_px(wmtk::get_value(u));
        const double px_y = param_to_px(wmtk::get_value(v));
        return m_patch.evaluate(px_x, px_y)[2];
    }

    DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d> sample(
        const DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>& u,
        const DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>& v) const
    {
        const double px_x = param_to_px(wmtk::get_value(u));
        const double px_y = param_to_px(wmtk::get_value(v));
        const double value = m_patch.evaluate(px_x, px_y)[2];

        const double du = m_patch.evaluate_derivative_u(px_x, px_y)[2];
        const double dv = m_patch.evaluate_derivative_v(px_x, px_y)[2];
        const Eigen::Vector2d grad = {du, dv};

        const double duu = m_patch.evaluate_2nd_derivative_uu(px_x, px_y)[2];
        const double dvv = m_patch.evaluate_2nd_derivative_vv(px_x, px_y)[2];
        const double duv = m_patch.evaluate_2nd_derivative_uv(px_x, px_y)[2];
        const Eigen::Matrix2d hess{{duu, duv}, {duv, dvv}};

        return {value, grad, hess};
    }

    Scalar operator()(const Scalar& u, const Scalar& v) { return sample(u, v); }
};

inline std::unique_ptr<Sampling> create_sampler(
    const Image& image,
    const SAMPLING_MODE sampling_mode)
{
    switch (sampling_mode) {
    case SAMPLING_MODE::BICUBIC: return std::make_unique<SamplingBicubic>(image);
    case SAMPLING_MODE::SPLINE: return std::make_unique<SamplingSpline>(image);
    default: break;
    }
    return {};
};
} // namespace wmtk
