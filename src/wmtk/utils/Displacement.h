#pragma once

#include <type_traits>
#include "Image.h"
#include "LineQuadrature.hpp"
#include "autodiff.h"
namespace wmtk {
class Displacement
{
public:
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;
    virtual ~Displacement(){};

public:
    virtual double get(double x, double y) const = 0;
    virtual DScalar get(const DScalar& x, const DScalar& y) const = 0;
    virtual double get_error_per_edge(
        const Eigen::Matrix<double, 2, 1>& p1,
        const Eigen::Matrix<double, 2, 1>& p2) const = 0;
    virtual DScalar get_error_per_edge(
        const Eigen::Matrix<DScalar, 2, 1>& p1,
        const Eigen::Matrix<DScalar, 2, 1>& p2) const = 0;
};

// Shared templated parent class
template <typename Derived>
class DisplacementImage : public Displacement
{
protected:
    wmtk::Image m_image;

public:
    DisplacementImage(const wmtk::Image img)
        : m_image(img)
    {
        assert(m_image.width() == m_image.height());
    }

public:
    double get(double x, double y) const override
    {
        return static_cast<const Derived*>(this)->get(x, y);
    }

    DScalar get(const DScalar& x, const DScalar& y) const override
    {
        return static_cast<const Derived*>(this)->get(x, y);
    }
    void set_image(const Image& image) { m_image = image; }

public:
    template <class T, int order>
    inline T quadrature_error_1pixel_eval(
        const Eigen::Matrix<T, 2, 3> edge_verts,
        wmtk::LineQuadrature& quad) const
    {
        quad.get_quadrature(order);
        T ret = static_cast<T>(0.0);
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
            auto tmph = get(tmpu, tmpv);
            auto tmpz = static_cast<T>(1 - quad.points(i, 0)) * v1z +
                        static_cast<T>(quad.points(i, 0)) * v2z;
            ret += static_cast<T>(quad.weights(i)) * abs(tmph - tmpz);
        }
        auto edge_norm = sqrt(
            (v12d(0) - v22d(0)) * (v12d(0) - v22d(0)) + (v12d(1) - v22d(1)) * (v12d(1) - v22d(1)));
        return ret * edge_norm;
    }

    inline double get_error_per_edge(
        const Eigen::Matrix<double, 2, 1>& p1,
        const Eigen::Matrix<double, 2, 1>& p2) const override
    {
        return get_error_per_edge_T(p1, p2);
    }

    inline DScalar get_error_per_edge(
        const Eigen::Matrix<DScalar, 2, 1>& p1,
        const Eigen::Matrix<DScalar, 2, 1>& p2) const override
    {
        return get_error_per_edge_T(p1, p2);
    }

    template <class T>
    inline T get_error_per_edge_T(
        const Eigen::Matrix<T, 2, 1>& p1,
        const Eigen::Matrix<T, 2, 1>& p2) const
    {
        auto v1z = get(p1(0), p1(1));
        auto v2z = get(p2(0), p2(1));
        // get the pixel index of p1 and p2
        auto get_coordinate = [&](const T& x, const T& y) -> std::pair<int, int> {
            auto [xx, yy] = m_image.get_pixel_index(get_value(x), get_value(y));
            return {m_image.get_coordinate(xx, m_image.get_wrapping_mode_x()),
                    m_image.get_coordinate(yy, m_image.get_wrapping_mode_y())};
        };
        auto [xx1, yy1] = get_coordinate(p1(0), p1(1));
        auto [xx2, yy2] = get_coordinate(p2(0), p2(1));
        // get all the pixels in between p1 and p2
        auto pixel_num = std::max(abs(xx2 - xx1), abs(yy2 - yy1));
        if (pixel_num <= 0) return static_cast<T>(0.);
        assert(pixel_num > 0);
        T error = static_cast<T>(0.0);
        for (int i = 0; i < pixel_num; i++) {
            const auto r0 = static_cast<T>(i) / pixel_num;
            auto tmp_p1 = p1 * (1. - r0) + p2 * r0;
            auto tmp_v1z = v1z * (1. - r0) + v2z * r0;
            const auto r1 = static_cast<T>(i + 1) / pixel_num;
            auto tmp_p2 = p1 * (1. - r1) + p2 * r1;
            auto tmp_v2z = v1z * (1. - r1) + v2z * r1;
            Eigen::Matrix<T, 2, 3> edge_verts;
            edge_verts.row(0) << tmp_p1(0), tmp_p1(1), tmp_v1z;
            edge_verts.row(1) << tmp_p2(0), tmp_p2(1), tmp_v2z;
            wmtk::LineQuadrature quad;
            auto displaced_error_per_pixel = quadrature_error_1pixel_eval<T, 5>(edge_verts, quad);
            error += displaced_error_per_pixel;
        }
        assert(error >= 0);
        return error;
    }
};
} // namespace wmtk