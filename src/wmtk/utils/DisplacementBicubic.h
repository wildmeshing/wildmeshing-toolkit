#pragma once

//#include "Image.h"
#include <wmtk/utils/LineQuadrature.hpp>
#include "Displacement.h"
#include "Image.h"
#include "bicubic_interpolation.h"

#include <Eigen/Core>


namespace wmtk {

class DisplacementBicubic : public Displacement
{
    using Scalar = double;
    Image img_; // displacement as pixel image

    int img_size_ = 0;
    double inv_img_size_ = 0;

    ::WrappingMode wrap_x_ = ::WrappingMode::MIRROR_REPEAT;
    ::WrappingMode wrap_y_ = ::WrappingMode::MIRROR_REPEAT;

public:
    DisplacementBicubic(
        const Image& img,
        const ::WrappingMode& wrap_x = ::WrappingMode::MIRROR_REPEAT,
        const ::WrappingMode& wrap_y = ::WrappingMode::MIRROR_REPEAT)
        : img_size_(img.width())
        , inv_img_size_(1.0 / img.width())
        , wrap_x_(wrap_x)
        , wrap_y_(wrap_y)
        , img_(img)
    {
        assert(img_.width() == img_.height());
        assert(img_.width() != 0);
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
            img_.get_raw_image().data(),
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

public:
    template <class T, int order>
    inline std::decay_t<T> quadrature_error_1pixel_eval(
        const Eigen::Matrix<T, 2, 3> edge_verts,
        wmtk::LineQuadrature& quad) const
    {
        quad.get_quadrature(order);
        double ret = 0.0;
        auto v1z = edge_verts(0, 2);
        auto v2z = edge_verts(1, 2);
        Eigen::Matrix<T, 1, 2> v12d, v22d;
        v12d << edge_verts(0, 0), edge_verts(0, 1);
        v22d << edge_verts(1, 0), edge_verts(1, 1);
        // now do 1d quadrature
        for (int i = 0; i < quad.points.rows(); i++) {
            auto tmpu =
                (1 - quad.points(i, 0)) * edge_verts(0, 0) + quad.points(i, 0) * edge_verts(1, 0);
            auto tmpv =
                (1 - quad.points(i, 0)) * edge_verts(0, 1) + quad.points(i, 0) * edge_verts(1, 1);
            auto tmph = DisplacementBicubic::get(tmpu, tmpv);
            auto tmpz = (1 - quad.points(i, 0)) * v1z + quad.points(i, 0) * v2z;

            wmtk::logger()
                .info("   displacement tmpu {} tmpv {} tmph {} tmpz {}", tmpu, tmpv, tmph, tmpz);
            ret += abs(quad.weights(i) * (tmph - tmpz));
        }
        wmtk::logger().info("   ret for {} {} in displacement is {}", v12d, v22d, ret);
        return ret * (v12d - v22d).stableNorm();
    }

    template <class T>
    inline std::decay_t<T> get_error_per_edge(
        const Eigen::Matrix<T, 2, 1>& p1,
        const Eigen::Matrix<T, 2, 1>& p2) const
    {
        auto v1z = DisplacementBicubic::get(p1(0), p1(1));
        auto v2z = DisplacementBicubic::get(p2(0), p2(1));
        // get the pixel index of p1 and p2
        auto get_coordinate = [&](const T& x, const T& y) -> std::pair<int, int> {
            auto [xx, yy] = img_.get_pixel_index(x, y);
            return {img_.get_coordinate(xx, wrap_x_), img_.get_coordinate(yy, wrap_y_)};
        };
        auto [xx1, yy1] = get_coordinate(p1(0), p1(1));
        auto [xx2, yy2] = get_coordinate(p2(0), p2(1));
        // get all the pixels in between p1 and p2
        auto pixel_num = std::max(abs(xx2 - xx1), abs(yy2 - yy1));
        if (pixel_num <= 0) return 0.;
        assert(pixel_num > 0);
        double error = 0.0;
        for (int i = 0; i < pixel_num; i++) {
            const double r0 = static_cast<double>(i) / pixel_num;
            auto tmp_p1 = p1 * (1. - r0) + p2 * r0;
            auto tmp_v1z = v1z * (1. - r0) + v2z * r0;
            const double r1 = static_cast<double>(i + 1) / pixel_num;
            auto tmp_p2 = p1 * (1. - r1) + p2 * r1;
            auto tmp_v2z = v1z * (1. - r1) + v2z * r1;
            Eigen::Matrix<T, 2, 3> edge_verts;
            edge_verts.row(0) << tmp_p1(0), tmp_p1(1), tmp_v1z;
            edge_verts.row(1) << tmp_p2(0), tmp_p2(1), tmp_v2z;
            LineQuadrature quad;
            auto displaced_pixel_error = quadrature_error_1pixel_eval<T, 5>(edge_verts, quad);
            error += displaced_pixel_error;
        }
        assert(error >= 0);
        return error;
    }
};
} // namespace wmtk
