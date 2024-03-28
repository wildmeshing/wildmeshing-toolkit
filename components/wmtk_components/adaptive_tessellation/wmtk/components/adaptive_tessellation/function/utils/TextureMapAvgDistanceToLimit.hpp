#pragma once


// #include <tbb/concurrent_vector.h>
// #include <tbb/enumerable_thread_specific.h>
// #include <tbb/parallel_for.h>

#include <array>
#include <iostream>
#include <wmtk/components/adaptive_tessellation/image/utils/sampling_utils.hpp>
#include <wmtk/components/adaptive_tessellation/quadrature/ClippedQuadrature.hpp>
#include <wmtk/components/adaptive_tessellation/quadrature/Quadrature.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/triangle_areas.hpp>

#include "IntegralBasedAvgDistance.hpp"
#include "ThreeChannelPositionMapEvaluator.hpp"

#include <nlohmann/json.hpp>
namespace image = wmtk::components::image;
namespace wmtk::components {
namespace function::utils {


class TextureMapAvgDistanceToLimit : public IntegralBasedAvgDistance
{
public:
    // using DScalar = DScalar2<double, Eigen::Matrix<double, -1, 1>, Eigen::Matrix<double, -1,
    // -1>>; using DTriangle = Eigen::Matrix<DScalar, 3, 2, Eigen::RowMajor>;

public:
    TextureMapAvgDistanceToLimit(); // default constructor
    TextureMapAvgDistanceToLimit(const TextureMapAvgDistanceToLimit&) = delete; // copy constructor
    TextureMapAvgDistanceToLimit(TextureMapAvgDistanceToLimit&&); // move constructor
    TextureMapAvgDistanceToLimit& operator=(const TextureMapAvgDistanceToLimit&) =
        delete; // copy assignment operator
    TextureMapAvgDistanceToLimit& operator=(
        TextureMapAvgDistanceToLimit&&); // move assignment operator
    ~TextureMapAvgDistanceToLimit(); // destructor
    TextureMapAvgDistanceToLimit(const ThreeChannelPositionMapEvaluator& evaluator);
    TextureMapAvgDistanceToLimit(const ThreeChannelPositionMapEvaluator& evaluator, bool debug);

public:
    double triangle_quadrature(
        const Vector2<double>& uv0,
        const Vector2<double>& uv1,
        const Vector2<double>& uv2) const override;
    DScalar triangle_quadrature(
        const Vector2<DScalar>& uv0,
        const Vector2<DScalar>& uv1,
        const Vector2<DScalar>& uv2) const override;
    ///
    /// Computes the error integral per triangle.
    ///
    /// Let q_img(.) be a texture mapping UV-space to the displaced positions in R^3.
    ///
    /// Given a UV-triangle f=(v0, v1, v2), a point m=(u, v) inside f will be mapped to the 3D
    /// position
    ///
    /// q_tri(u, v) = λ * q_v0 + μ * q_v1 + (1 - λ - μ) * q_v2
    ///
    /// where:
    /// - λ and μ are the barycentric coordinates of m inside the triangle f.
    /// - q_vi are the 3D positions of the triangle corners v0, v1, v2 according the q_img(.)
    ///   formula above.
    ///
    /// The "error" function is then defined as the integral of the distance between q_img(u, v) and
    /// q_tri(u, v) over the triangle f:
    ///
    /// error(f) = ∫_{u,v \in f} ‖q_tri(u, v) - q_img(u, v)‖^2 du dv
    ///
    /// @param[in]  input_triangles  UV coordinates of each input triangle corners (u0, v0, u1, v1,
    ///                              u2, v2).
    /// @param[in]  output_errors    A pre-allocated buffer where to store the error integral for
    ///                              each input triangle.
    ///
    // void get_error_per_triangle(
    //     lagrange::span<const std::array<float, 6>> input_triangles,
    //     lagrange::span<float> output_errors) const;

    template <typename T>
    T triangle_quadrature_T(const Vector2<T>& uv0, const Vector2<T>& uv1, const Vector2<T>& uv2)
        const
    {
        constexpr int Degree = 4;
        // const int order = 2 * (Degree - 1);
        auto cache = m_cache->quadrature_cache;

        Vector3<T> p0, p1, p2;
        p0 = m_three_channel_evaluator.uv_to_position(uv0);
        p1 = m_three_channel_evaluator.uv_to_position(uv1);
        p2 = m_three_channel_evaluator.uv_to_position(uv2);

        Eigen::Matrix<T, 3, 3, Eigen::ColMajor> position_triangle_ColMajor;
        position_triangle_ColMajor.col(0) = p0;
        position_triangle_ColMajor.col(1) = p1;
        position_triangle_ColMajor.col(2) = p2;

        Eigen::Matrix<double, 3, 2, RowMajor> uv_triangle_RowMajor;
        uv_triangle_RowMajor.row(0) = image::utils::get_double(uv0);
        uv_triangle_RowMajor.row(1) = image::utils::get_double(uv1);
        uv_triangle_RowMajor.row(2) = image::utils::get_double(uv2);

        // std::cout << "uv0 " << uv_triangle_RowMajor.row(0) << std::endl;
        // std::cout << "uv1 " << uv_triangle_RowMajor.row(1) << std::endl;
        // std::cout << "uv2 " << uv_triangle_RowMajor.row(2) << std::endl;

        // calculate the barycentric coordinate of the a point using u, v cooridnates
        // returns the 3d coordinate on the current mesh
        BarycentricTriangle<T> bary(uv0, uv1, uv2);
        if (bary.is_degenerate()) {
            return T(0.);
        }

        auto squared_norm_T = [&](const Eigen::Matrix<T, 3, 1>& row_v) -> T {
            T ret = T(0.);
            for (auto i = 0; i < row_v.rows(); i++) {
                ret += pow(row_v(i, 0), 2);
            }
            return pow(ret, 0.5);
        };
        T value = T(0.);
        Eigen::AlignedBox2d bbox = uv_triangle_bbox(uv_triangle_RowMajor);
        auto [num_pixels, pixel_size] = pixel_num_size_of_uv_triangle(bbox);

        for (auto y = 0; y < num_pixels; ++y) {
            for (auto x = 0; x < num_pixels; ++x) {
                Eigen::AlignedBox2d box;
                box.extend(bbox.min() + Eigen::Vector2d(x * pixel_size, y * pixel_size));
                box.extend(
                    bbox.min() + Eigen::Vector2d((x + 1) * pixel_size, (y + 1) * pixel_size));
                wmtk::ClippedQuadrature rules;
                rules.clipped_triangle_box_quadrature(
                    Degree + 1,
                    uv_triangle_RowMajor,
                    box,
                    cache.quad,
                    &cache.tmp);
                // cache.local().quad,
                // &cache.local().tmp);
                for (auto i = 0; i < cache.quad.size(); ++i) {
                    Vector2<T> quad_point_uv = cache.quad.points().row(i).template cast<T>();
                    Vector3<T> texture_position =
                        m_three_channel_evaluator.uv_to_position(quad_point_uv);
                    Vector3<T> position = position_triangle_ColMajor * bary.get(quad_point_uv);
                    Vector3<T> diffp = texture_position - position;
                    value += squared_norm_T(diffp) * T(cache.quad.weights()[i]);
                }
            }
        }
        // // scaling by jacobian
        // value = value * wmtk::utils::triangle_3d_area(p0, p1, p2);
        // value = value / wmtk::utils::triangle_unsigned_2d_area(uv0, uv1, uv2);
        return value;
    }

protected:
    template <typename T>
    Eigen::AlignedBox2d
    uv_triangle_bbox(const Vector2<T>& uv0, const Vector2<T>& uv1, const Vector2<T>& uv2) const
    {
        Eigen::AlignedBox2d bbox;
        bbox.extend(Vector2d(image::utils::get_double(uv0)));
        bbox.extend(Vector2d(image::utils::get_double(uv1)));
        bbox.extend(Vector2d(image::utils::get_double(uv2)));
        return bbox;
    }


    Eigen::AlignedBox2d uv_triangle_bbox(
        const Eigen::Matrix<double, 3, 2, RowMajor>& uv_triangle_RowMajor) const
    {
        Eigen::AlignedBox2d bbox;
        bbox.extend(uv_triangle_RowMajor.row(0).transpose());
        bbox.extend(uv_triangle_RowMajor.row(1).transpose());
        bbox.extend(uv_triangle_RowMajor.row(2).transpose());
        return bbox;
    }
    std::pair<int, double> pixel_num_size_of_uv_triangle(Eigen::AlignedBox2d& bbox) const;

    std::pair<int, double> pixel_size_of_uv_triangle(int pixel_num, Eigen::AlignedBox2d& bbox)
        const;

protected:
    std::shared_ptr<Cache> m_cache;
    bool m_debug;

public:
    mutable nlohmann::ordered_json m_jsonData_bary_coord,
        m_jsonData_texture_coord; // this is just for debug
};
} // namespace function::utils
} // namespace wmtk::components
