#include "TextureIntegral.hpp"

#include <wmtk/components/adaptive_tessellation/quadrature/ClippedQuadrature.hpp>
#include <wmtk/utils/triangle_areas.hpp>
#include "BarycentricTriangle.hpp"

using namespace Eigen;
namespace wmtk::components {
namespace function::utils {

TextureIntegral::TextureIntegral(const ThreeChannelPositionMapEvaluator& evaluator)
    : m_three_channel_evaluator(evaluator)
    , m_cache(std::make_shared<Cache>())
{
    std::cout << "==== TextureIntegral constructor" << std::endl;
}

TextureIntegral::TextureIntegral(const ThreeChannelPositionMapEvaluator& evaluator, bool debug)
    : m_three_channel_evaluator(evaluator)
    , m_cache(std::make_shared<Cache>())
    , m_debug(debug)
{
    if (m_debug) {
        std::cout << "==== TextureIntegral with debug" << std::endl;
        m_jsonData_bary_coord = nlohmann::ordered_json();
        m_jsonData_texture_coord = nlohmann::ordered_json();
    }
}

TextureIntegral::~TextureIntegral() = default;

std::pair<int, double> TextureIntegral::pixel_num_size_of_uv_triangle(
    Eigen::AlignedBox2d& bbox) const
{
    Vector2d bbox_min = bbox.min();
    Vector2d bbox_max = bbox.max();
    auto [xx1, yy1] = m_three_channel_evaluator.pixel_index(bbox_min);
    auto [xx2, yy2] = m_three_channel_evaluator.pixel_index(bbox_max);

    int num_pixels = std::max(abs(xx2 - xx1), abs(yy2 - yy1)) + 1;
    assert(num_pixels > 0);
    double pixel_size = bbox.diagonal().maxCoeff() / num_pixels;
    return {num_pixels, pixel_size};
}

std::pair<int, double> TextureIntegral::pixel_size_of_uv_triangle(
    int pixel_num,
    Eigen::AlignedBox2d& bbox) const
{
    assert(pixel_num > 0);
    double pixel_size = bbox.diagonal().maxCoeff() / pixel_num;
    return {pixel_num, pixel_size};
}

double TextureIntegral::get_error_one_triangle_exact(
    const Vector2<double>& uv0,
    const Vector2<double>& uv1,
    const Vector2<double>& uv2) const
{
    constexpr int Degree = 4;
    // const int order = 2 * (Degree - 1);
    auto cache = m_cache->quadrature_cache;

    Vector3<double> p0, p1, p2;
    p0 = m_three_channel_evaluator.uv_to_position(uv0);
    p1 = m_three_channel_evaluator.uv_to_position(uv1);
    p2 = m_three_channel_evaluator.uv_to_position(uv2);

    Eigen::Matrix<double, 3, 3, Eigen::ColMajor> position_triangle_ColMajor;
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
    BarycentricTriangle<double> bary(uv0, uv1, uv2);
    if (bary.is_degenerate()) {
        return 0.;
    }

    auto squared_norm_T = [&](const Eigen::Matrix<double, 3, 1>& row_v) -> double {
        double ret = 0.;
        for (auto i = 0; i < row_v.rows(); i++) {
            ret += pow(row_v(i, 0), 4);
        }
        return ret;
    };
    double value = 0.;
    Eigen::AlignedBox2d bbox = uv_triangle_bbox(uv_triangle_RowMajor);
    // auto [num_pixels, pixel_size] = pixel_num_size_of_uv_triangle(bbox);
    auto [num_pixels, pixel_size] = pixel_size_of_uv_triangle(10, bbox);

    wmtk::logger().info("num_pixels {} pixel_size {}", num_pixels, pixel_size);
    for (auto y = 0; y < num_pixels; ++y) {
        for (auto x = 0; x < num_pixels; ++x) {
            Eigen::AlignedBox2d box;
            box.extend(bbox.min() + Eigen::Vector2d(x * pixel_size, y * pixel_size));
            box.extend(bbox.min() + Eigen::Vector2d((x + 1) * pixel_size, (y + 1) * pixel_size));
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
                Vector2<double> quad_point_uv = cache.quad.points().row(i);
                Vector3<double> texture_position =
                    m_three_channel_evaluator.uv_to_position(quad_point_uv);
                Vector3<double> position = position_triangle_ColMajor * bary.get(quad_point_uv);

                if (m_debug) {
                    m_jsonData_bary_coord.push_back(
                        {{"x", position(0)}, {"y", position(1)}, {"z", position(2)}});
                    m_jsonData_texture_coord.push_back(
                        {{"x", texture_position(0)},
                         {"y", texture_position(1)},
                         {"z", texture_position(2)}});
                }
                Vector3<double> diffp = texture_position - position;
                value += squared_norm_T(diffp) * cache.quad.weights()[i];
            }
        }
    }
    // // scaling by jacobian
    // value = value * wmtk::utils::triangle_3d_area(p0, p1, p2);
    // value = value / wmtk::utils::triangle_unsigned_2d_area(uv0, uv1, uv2);
    return value;
}
DScalar TextureIntegral::get_error_one_triangle_exact(
    const Vector2<DScalar>& uv0,
    const Vector2<DScalar>& uv1,
    const Vector2<DScalar>& uv2) const
{
    return get_error_one_triangle_exact_T<DScalar>(uv0, uv1, uv2);
}
} // namespace function::utils
} // namespace wmtk::components