#pragma once

#include <tbb/enumerable_thread_specific.h>
#include <wmtk/quadrature/ClippedQuadrature.h>
#include <wmtk/quadrature/TriangleQuadrature.h>
#include <wmtk/utils/PolygonClipping.h>
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
    virtual double get_error_per_triangle(
        const Eigen::Matrix<double, 3, 2, Eigen::RowMajor>& triangle) = 0;
    virtual DScalar get_error_per_triangle(
        const Eigen::Matrix<DScalar, 3, 2, Eigen::RowMajor>& triangle) = 0;
};

// Shared templated parent class
template <typename Derived>
class DisplacementImage : public Displacement
{
protected:
    wmtk::Image m_image;

public:
    struct QuadrCache
    {
        wmtk::Quadrature quad;
        wmtk::Quadrature tmp;
    };
    tbb::enumerable_thread_specific<QuadrCache> cache;

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

    inline double get_error_per_triangle(
        const Eigen::Matrix<double, 3, 2, Eigen::RowMajor>& triangle) override
    {
        return get_error_per_triangle_T(triangle);
    }

    inline DScalar get_error_per_triangle(
        const Eigen::Matrix<DScalar, 3, 2, Eigen::RowMajor>& triangle) override
    {
        return get_error_per_triangle_T(triangle);
    }

    template <class T>
    inline T get_error_per_triangle_T(const Eigen::Matrix<T, 3, 2, Eigen::RowMajor>& triangle)
    {
        constexpr int Degree = 4;
        const int order = 2 * (Degree - 1);
        Eigen::AlignedBox2d bbox;
        Eigen::Matrix<double, 3, 2, 1> triangle_double;
        for (auto i = 0; i < 3; i++) {
            Eigen::Vector2d p_double;
            p_double << get_value(triangle(i, 0)), get_value(triangle(i, 1));
            triangle_double.row(i) << p_double(0), p_double(1);
            bbox.extend(p_double);
        }
        auto get_coordinate = [&](const double& x, const double& y) -> std::pair<int, int> {
            auto [xx, yy] = m_image.get_pixel_index(get_value(x), get_value(y));
            return {m_image.get_coordinate(xx, m_image.get_wrapping_mode_x()),
                    m_image.get_coordinate(yy, m_image.get_wrapping_mode_y())};
        };
        auto bbox_min = bbox.min();
        auto bbox_max = bbox.max();
        auto [xx1, yy1] = get_coordinate(bbox_min(0), bbox_min(1));
        auto [xx2, yy2] = get_coordinate(bbox_max(0), bbox_max(1));
        auto num_pixels = std::max(abs(xx2 - xx1), abs(yy2 - yy1));
        const double pixel_size = bbox.diagonal().maxCoeff() / num_pixels;
        typedef std::integral_constant<int, 2> cached_t;

        auto z_barycentric = [&](const T& u, const T& v) -> T {
            /*
            λ1 = ((v2 - v3)(u - u3) + (u3 - u2)(v - v3)) / ((v2 - v3)(u1 - u3) + (u3 - u2)(v1 - v3))
            λ2 = ((v3 - v1)(u - u3) + (u1 - u3)(v - v3)) / ((v2 - v3)(u1 - u3) + (u3 - u2)(v1 - v3))
            λ3 = 1 - λ1 - λ2
            z = λ1 * z1 + λ2 * z2 + λ3 * z3
            */
            auto u1 = triangle(0, 0);
            auto v1 = triangle(0, 1);
            auto u2 = triangle(1, 0);
            auto v2 = triangle(1, 1);
            auto u3 = triangle(2, 0);
            auto v3 = triangle(2, 1);

            auto lambda1 = ((v2 - v3) * (u - u3) + (u3 - u2) * (v - v3)) /
                           ((v2 - v3) * (u1 - u3) + (u3 - u2) * (v1 - v3));
            auto lambda2 = ((v3 - v1) * (u - u3) + (u1 - u3) * (v - v3)) /
                           ((v2 - v3) * (u1 - u3) + (u3 - u2) * (v1 - v3));
            auto lambda3 = 1 - lambda1 - lambda2;
            return (
                lambda1 * m_image.get(u1, v1) + lambda2 * m_image.get(u2, v2) +
                lambda3 * m_image.get(u3, v3));
        };

        T value = static_cast<T>(0.);
        for (size_t x = 0; x < num_pixels; ++x) {
            for (size_t y = 0; y < num_pixels; ++y) {
                Eigen::AlignedBox2d box;
                box.extend(bbox.min() + Eigen::Vector2d(x * pixel_size, y * pixel_size));
                box.extend(
                    bbox.min() + Eigen::Vector2d((x + 1) * pixel_size, (y + 1) * pixel_size));
                wmtk::ClippedQuadrature rules;
                rules.clipped_triangle_box_quadrature(
                    order,
                    triangle_double,
                    box,
                    cache.local().quad,
                    &cache.local().tmp);
                for (size_t i = 0; i < cache.local().quad.size(); ++i) {
                    auto tmpu = static_cast<T>(cache.local().quad.points()(i, 0));
                    auto tmpv = static_cast<T>(cache.local().quad.points()(i, 1));
                    auto tmph = get(tmpu, tmpv);
                    auto tmpz = z_barycentric(tmpu, tmpv);
                    // wmtk::logger().info("           u {} v {}", tmpu, tmpv);
                    // wmtk::logger().info("           tmph {} tmpz {}", tmph, tmpz);
                    value += abs(tmph - tmpz) * static_cast<T>(cache.local().quad.weights()[i]);
                }
            }
        }
        return value;
    }
};
} // namespace wmtk