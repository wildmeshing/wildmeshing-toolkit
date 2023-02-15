#pragma once

//#include "Image.h"
#include "Displacement.h"

#include <nanospline/BSplinePatch.h>
#include <nanospline/arc_length.h>
#include <Eigen/Core>


namespace wmtk {

constexpr int SPLINE_DEGREE = 3;

class DisplacementSpline : public Displacement
{
    using Scalar = double;

    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        img_; // displacement as pixel image

    int img_size_ = 0;
    double inv_img_size_ = 0;

    ::WrappingMode wrap_x_ = ::WrappingMode::CLAMP_TO_EDGE;
    ::WrappingMode wrap_y_ = ::WrappingMode::CLAMP_TO_EDGE;

    int n_grid_points_ = 0;
    int n_control_points_ = 0;
    int pxl_offset = ((SPLINE_DEGREE - 1) / 2);
    double inv_n_grid_points_ = 0;

    Eigen::MatrixX<Scalar> control_mat_;

    nanospline::BSplinePatch<Scalar, 3, SPLINE_DEGREE, SPLINE_DEGREE> patch_;

public:
    DisplacementSpline(
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

        n_grid_points_ = img_size_;
        inv_n_grid_points_ = inv_img_size_;
        n_control_points_ = n_grid_points_ + SPLINE_DEGREE - 1;

        // create knot vector
        const int n_knots = n_control_points_ + SPLINE_DEGREE + 1;
        Eigen::VectorX<Scalar> knots;
        knots.resize(n_knots);
        for (int i = 0; i < n_knots; ++i) {
            knots[i] = i - SPLINE_DEGREE;
        }

        // create control matrix
        control_mat_.resize(n_control_points_ * n_control_points_, 3);
        for (int i = 0; i < n_control_points_; ++i) {
            const int i_px = cp_to_px(i);
            const int img_i_px = img.get_coordinate(i_px, wrap_x_);
            for (int j = 0; j < n_control_points_; ++j) {
                const int j_px = cp_to_px(j);
                const int img_j_px = img.get_coordinate(j_px, wrap_y_);
                control_mat_.row(j * n_control_points_ + i) =
                    Eigen::Matrix<Scalar, 1, 3>(i_px, j_px, img_(img_i_px, img_j_px));
            }
        }

        patch_.set_control_grid(control_mat_);

        patch_.set_knots_u(knots);
        patch_.set_knots_v(knots);
        patch_.initialize();
    }

    // Transformation functions
    int cp_to_px(const int& i_cp) const { return i_cp - pxl_offset; }
    double px_to_param(const double& px_x) const { return (px_x + 0.5) * inv_n_grid_points_; }
    double param_to_px(const double& u) const { return u * n_grid_points_ - 0.5; }
    double cp_to_param(const int& i_cp) const { return px_to_param(cp_to_px(i_cp)); }

    // Assume input function is defined on the unit square: u,v in [0,1].
    void set(
        const std::function<Scalar(const Scalar&, const Scalar&)>& f,
        const Displacement::WrappingMode& mode_x,
        const Displacement::WrappingMode& mode_y)
    {
        for (int i_cp = 0; i_cp < n_control_points_; ++i_cp) {
            const double u = cp_to_param(i_cp);
            for (int j_cp = 0; j_cp < n_control_points_; ++j_cp) {
                const double v = cp_to_param(j_cp);
                const double w = f(u, v);
                control_mat_.row(i_cp * n_control_points_ + j_cp)[2] = w;
            }
        }
    }

    template <class T>
    std::decay_t<T> get(const T& u, const T& v) const
    {
        const double px_x = param_to_px(wmtk::get_value(u));
        const double px_y = param_to_px(wmtk::get_value(v));
        return patch_.evaluate(px_x, px_y)[2];
    }

    Scalar operator()(const Scalar& u, const Scalar& v) { return get(u, v); }

private:
};

} // namespace wmtk
