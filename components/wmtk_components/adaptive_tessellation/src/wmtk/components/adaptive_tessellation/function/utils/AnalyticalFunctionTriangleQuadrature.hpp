#pragma once


// #include <tbb/concurrent_vector.h>
// #include <tbb/enumerable_thread_specific.h>
// #include <tbb/parallel_for.h>

#include <array>
#include <iostream>
#include <wmtk/components/adaptive_tessellation/image/utils/sampling_utils.hpp>
#include <wmtk/components/adaptive_tessellation/quadrature/Quadrature.hpp>
#include <wmtk/components/adaptive_tessellation/quadrature/TriangleQuadrature.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/triangle_areas.hpp>
#include "BarycentricTriangle.hpp"
#include "IntegralBase.hpp"
#include "ThreeChannelPositionMapEvaluator.hpp"
namespace image = wmtk::components::adaptive_tessellation::image;
namespace wmtk::components::adaptive_tessellation {
namespace function::utils {


class AnalyticalFunctionTriangleQuadrature : public IntegralBase
{
public:
    using DScalar = DScalar2<double, Eigen::Matrix<double, -1, 1>, Eigen::Matrix<double, -1, -1>>;
    using DTriangle = Eigen::Matrix<DScalar, 3, 2, Eigen::RowMajor>;

public:
    AnalyticalFunctionTriangleQuadrature(); // default constructor
    AnalyticalFunctionTriangleQuadrature(const AnalyticalFunctionTriangleQuadrature&) =
        delete; // copy constructor
    AnalyticalFunctionTriangleQuadrature(
        AnalyticalFunctionTriangleQuadrature&&); // move constructor
    AnalyticalFunctionTriangleQuadrature& operator=(const AnalyticalFunctionTriangleQuadrature&) =
        delete; // copy assignment operator
    AnalyticalFunctionTriangleQuadrature& operator=(
        AnalyticalFunctionTriangleQuadrature&&); // move assignment operator
    ~AnalyticalFunctionTriangleQuadrature(); // destructor

    AnalyticalFunctionTriangleQuadrature(const ThreeChannelPositionMapEvaluator& evaluator);

public:
    template <typename T>
    T get_error_one_triangle_exact(
        const Vector2<T>& uv0,
        const Vector2<T>& uv1,
        const Vector2<T>& uv2) const
    {
        constexpr int Degree = 6;
        // const int order = 2 * (Degree - 1);
        // auto cache = m_cache->quadrature_cache;

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

        std::cout << "uv0 " << uv_triangle_RowMajor.row(0) << std::endl;
        std::cout << "uv1 " << uv_triangle_RowMajor.row(1) << std::endl;
        std::cout << "uv2 " << uv_triangle_RowMajor.row(2) << std::endl;

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
            return ret;
        };
        T value = T(0.);


        Quadrature tmp;
        TriangleQuadrature rules;
        rules.transformed_triangle_quadrature(Degree + 1, uv_triangle_RowMajor, tmp);

        for (auto i = 0; i < tmp.size(); ++i) {
            Vector2<T> quad_point_uv = tmp.points().row(i).template cast<T>();
            Vector3<T> texture_position = m_three_channel_evaluator.uv_to_position(quad_point_uv);
            Vector3<T> position = position_triangle_ColMajor * bary.get(quad_point_uv);
            Vector3<T> diffp = texture_position - position;
            value += squared_norm_T(diffp) * T(tmp.weights()[i]);
        }
        // scaling by jacobian
        value = value * wmtk::utils::triangle_3d_area(p0, p1, p2);
        value = value / wmtk::utils::triangle_unsigned_2d_area(uv0, uv1, uv2);
        return value;
    }

protected:
    const ThreeChannelPositionMapEvaluator& m_three_channel_evaluator;
};
} // namespace function::utils
} // namespace wmtk::components::adaptive_tessellation
