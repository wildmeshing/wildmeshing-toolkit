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
    virtual Eigen::Matrix<double, 3, 1> get(double x, double y) const = 0;
    virtual Eigen::Matrix<DScalar, 3, 1> get(const DScalar& x, const DScalar& y) const = 0;
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
// class DisplacementImage : public Displacement
// {
//     get_error_per_edge;
//     sampler;
// };

// class DisplacementMesh : public DisplacementImage
// {
// protected:
//     std::array<wmtk::Image, 3> m_position_image;
//     std::array<wmtk::Image, 3> m_normals_image;
// };
// Shared templated parent class
// class DisplacementPlane : public DisplacementImage
template <typename Derived>
class DisplacementImage : public Displacement
{
protected:
    wmtk::Image m_image;

protected:
    struct QuadrCache
    {
        wmtk::Quadrature quad;
        wmtk::Quadrature tmp;
    };
    tbb::enumerable_thread_specific<QuadrCache> m_cache;

public:
    DisplacementImage(const wmtk::Image img)
        : m_image(img)
    {
        assert(m_image.width() == m_image.height());
    }

public:
    Eigen::Matrix<double, 3, 1> get(double x, double y) const override
    {
        auto z = static_cast<const Derived*>(this)->get_height(x, y);
        Eigen::Matrix<double, 3, 1> coord_3d;
        coord_3d << x, y, z;
        return coord_3d;
    }

    Eigen::Matrix<DScalar, 3, 1> get(const DScalar& x, const DScalar& y) const override
    {
        auto z = static_cast<const Derived*>(this)->get_height(x, y);
        Eigen::Matrix<DScalar, 3, 1> coord_3d;
        coord_3d << x, y, z;
        return coord_3d;
    }
    void set_image(const Image& image) { m_image = image; }

public:
    inline double get_error_per_edge(
        const Eigen::Matrix<double, 2, 1>& p1,
        const Eigen::Matrix<double, 2, 1>& p2) const override
    {
        return get_error_per_edge_T<double, 5>(p1, p2);
    }

    inline DScalar get_error_per_edge(
        const Eigen::Matrix<DScalar, 2, 1>& p1,
        const Eigen::Matrix<DScalar, 2, 1>& p2) const override
    {
        return get_error_per_edge_T<DScalar, 5>(p1, p2);
    }

    template <class T, int order>
    inline T get_error_per_edge_T(
        const Eigen::Matrix<T, 2, 1>& uv1,
        const Eigen::Matrix<T, 2, 1>& uv2) const
    {
        auto p1_displaced = get(uv1(0), uv1(1));
        auto p2_displaced = get(uv2(0), uv2(1));
        // get the pixel index of p1 and p2
        auto get_coordinate = [&](const T& x, const T& y) -> std::pair<int, int> {
            auto [xx, yy] = m_image.get_pixel_index(get_value(x), get_value(y));
            return {m_image.get_coordinate(xx, m_image.get_wrapping_mode_x()),
                    m_image.get_coordinate(yy, m_image.get_wrapping_mode_y())};
        };
        auto [xx1, yy1] = get_coordinate(uv1(0), uv1(1));
        auto [xx2, yy2] = get_coordinate(uv2(0), uv2(1));
        // get all the pixels in between p1 and p2
        auto pixel_num = std::max(abs(xx2 - xx1), abs(yy2 - yy1));
        if (pixel_num <= 0) return static_cast<T>(0.);
        assert(pixel_num > 0);
        T error = static_cast<T>(0.0);
        auto norm_T = [&](const Eigen::Matrix<T, 1, Eigen::Dynamic> row_v) -> T {
            T ret = static_cast<T>(0.);
            for (auto i = 0; i < row_v.cols(); i++) {
                auto debug_rowv = row_v(0, i);
                ret += pow(row_v(0, i), 2);
            }
            return sqrt(ret);
        };
        for (int i = 0; i < pixel_num; i++) {
            const auto r0 = static_cast<T>(i) / pixel_num;
            const auto r1 = static_cast<T>(i + 1) / pixel_num;
            Eigen::Matrix<T, 2, 1> pixel_uv1 = uv1 * (1. - r0) + uv2 * r0;
            Eigen::Matrix<T, 2, 1> pixel_uv2 = uv1 * (1. - r1) + uv2 * r1;
            Eigen::Matrix<T, 3, 1> pixel_p1 = p1_displaced * (1. - r0) + p2_displaced * r0;
            Eigen::Matrix<T, 3, 1> pixel_p2 = p1_displaced * (1. - r1) + p2_displaced * r1;
            wmtk::LineQuadrature quad;
            quad.get_quadrature(order);

            T unweighted_error = static_cast<T>(0.0);
            for (int j = 0; j < quad.points.rows(); j++) {
                Eigen::Matrix<T, 2, 1> tmpuv = static_cast<T>(1 - quad.points(j, 0)) * pixel_uv1 +
                                               static_cast<T>(quad.points(j, 0)) * pixel_uv2;
                Eigen::Matrix<T, 3, 1> tmpp_displaced = get(tmpuv(0), tmpuv(1));

                Eigen::Matrix<T, 3, 1> tmpp_tri = static_cast<T>(1 - quad.points(j, 0)) * pixel_p1 +
                                                  static_cast<T>(quad.points(j, 0)) * pixel_p2;
                Eigen::Matrix<T, 3, 1> diffp = tmpp_tri - tmpp_displaced;
                unweighted_error += static_cast<T>(quad.weights(j)) * norm_T(diffp);
            }
            auto diffuv = pixel_uv1 - pixel_uv2;
            error += unweighted_error * norm_T(diffuv);
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

    /**
     * @brief Get the error per triangle T object
     *
     * @tparam T
     * @param triangle 3 x 2 matrix of triangle 3 vertices 2d uv coordinates
     * @return T
     */
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
        auto norm_T = [&](const Eigen::Matrix<T, 1, Eigen::Dynamic> row_v) -> T {
            T ret = static_cast<T>(0.);
            for (auto i = 0; i < row_v.cols(); i++) {
                auto debug_rowv = row_v(0, i);
                ret += pow(row_v(0, i), 2);
            }
            return sqrt(ret);
        };
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

        T z1 = m_image.get(triangle(0, 0), triangle(0, 1));
        T z2 = m_image.get(triangle(1, 0), triangle(1, 1));
        T z3 = m_image.get(triangle(2, 0), triangle(2, 1));
        wmtk::logger().info("x1 {} y1 {} z1 {}", triangle(0, 0), triangle(0, 1), z1);
        wmtk::logger().info("x2 {} y2 {} z2 {}", triangle(1, 0), triangle(1, 1), z2);
        wmtk::logger().info("x3 {} y3 {} z3 {}", triangle(2, 0), triangle(2, 1), z3);
        typedef std::integral_constant<int, 2> cached_t;

        // calculate the barycentric coordinate of the a point using u, v cooridnates
        // returns the 3d coordinate on the current mesh
        auto get_p_tri = [&](const T& u, const T& v) -> Eigen::Matrix<T, 3, 1> {
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
            auto z = (lambda1 * z1 + lambda2 * z2 + lambda3 * z3);
            Eigen::Matrix<T, 3, 1> p_tri;
            p_tri << u, v, z;
            return p_tri;
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
                    m_cache.local().quad,
                    &m_cache.local().tmp);
                for (size_t i = 0; i < m_cache.local().quad.size(); ++i) {
                    auto tmpu = static_cast<T>(m_cache.local().quad.points()(i, 0));
                    auto tmpv = static_cast<T>(m_cache.local().quad.points()(i, 1));
                    auto tmpp_displaced = get(tmpu, tmpv);
                    auto tmpp_tri = get_p_tri(tmpu, tmpv);
                    // wmtk::logger().info("           u {} v {}", tmpu, tmpv);
                    // wmtk::logger().info("           tmph {} tmpz {}", tmph, tmpz);
                    auto diffp = tmpp_displaced - tmpp_tri;
                    value += norm_T(diffp) * static_cast<T>(m_cache.local().quad.weights()[i]);
                }
            }
        }
        return value;
    }
};
} // namespace wmtk