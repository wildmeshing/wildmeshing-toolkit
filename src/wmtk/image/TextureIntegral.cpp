#include "TextureIntegral.h"

#include <wmtk/quadrature/ClippedQuadrature.h>
#include <wmtk/quadrature/TriangleQuadrature.h>
#include <wmtk/utils/Sampling.h>

#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

namespace wmtk {

namespace {

struct QuadratureCache
{
    wmtk::Quadrature quad;
    wmtk::Quadrature tmp;
};

// Reference implementation copied over from Displacement.h
template <typename DisplacementFunc>
double get_error_per_triangle_exact(
    const wmtk::Image& m_image,
    const Eigen::Matrix<double, 3, 2, Eigen::RowMajor>& triangle,
    tbb::enumerable_thread_specific<QuadratureCache>& m_cache,
    DisplacementFunc get)
{
    using T = double;
    constexpr int Degree = 4;
    const int order = 2 * (Degree - 1);

    Eigen::Matrix<T, 3, 1> p1 = get(triangle(0, 0), triangle(0, 1));
    Eigen::Matrix<T, 3, 1> p2 = get(triangle(1, 0), triangle(1, 1));
    Eigen::Matrix<T, 3, 1> p3 = get(triangle(2, 0), triangle(2, 1));
    Eigen::AlignedBox2d bbox;
    Eigen::Matrix<double, 3, 2, 1> triangle_double;
    for (auto i = 0; i < 3; i++) {
        Eigen::Vector2d p_double;
        p_double << get_value(triangle(i, 0)), get_value(triangle(i, 1));
        triangle_double.row(i) << p_double(0), p_double(1);
        bbox.extend(p_double);
    }
    auto squared_norm_T = [&](const Eigen::Matrix<T, 3, 1>& row_v) -> T {
        T ret = T(0.);
        for (auto i = 0; i < row_v.rows(); i++) {
            auto debug_rowv = row_v(i, 0);
            ret += pow(row_v(i, 0), 2);
        }
        return ret;
    };
    auto get_coordinate = [&](const double& x, const double& y) -> std::pair<int, int> {
        auto [xx, yy] = m_image.get_pixel_index(get_value(x), get_value(y));
        return {
            m_image.get_coordinate(xx, m_image.get_wrapping_mode_x()),
            m_image.get_coordinate(yy, m_image.get_wrapping_mode_y())};
    };
    auto bbox_min = bbox.min();
    auto bbox_max = bbox.max();
    auto [xx1, yy1] = get_coordinate(bbox_min(0), bbox_min(1));
    auto [xx2, yy2] = get_coordinate(bbox_max(0), bbox_max(1));
    auto num_pixels = std::max(abs(xx2 - xx1), abs(yy2 - yy1)) + 1;
    assert(num_pixels > 0);
    const double pixel_size = bbox.diagonal().maxCoeff() / num_pixels;
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
        Eigen::Matrix<T, 3, 1> p_tri = (lambda1 * p1 + lambda2 * p2 + lambda3 * p3);
        return p_tri;
    };

    auto check_degenerate = [&]() -> bool {
        auto u1 = triangle(0, 0);
        auto v1 = triangle(0, 1);
        auto u2 = triangle(1, 0);
        auto v2 = triangle(1, 1);
        auto u3 = triangle(2, 0);
        auto v3 = triangle(2, 1);
        if ((v2 - v3) * (u1 - u3) + (u3 - u2) * (v1 - v3) == 0.) return false;
        if ((v2 - v3) * (u1 - u3) + (u3 - u2) * (v1 - v3) == 0.) return false;
        return true;
    };

    T value = T(0.);
    for (auto x = 0; x < num_pixels; ++x) {
        for (auto y = 0; y < num_pixels; ++y) {
            Eigen::AlignedBox2d box;
            box.extend(bbox.min() + Eigen::Vector2d(x * pixel_size, y * pixel_size));
            box.extend(bbox.min() + Eigen::Vector2d((x + 1) * pixel_size, (y + 1) * pixel_size));
            wmtk::ClippedQuadrature rules;
            rules.clipped_triangle_box_quadrature(
                order,
                triangle_double,
                box,
                m_cache.local().quad,
                &m_cache.local().tmp);
            for (auto i = 0; i < m_cache.local().quad.size(); ++i) {
                auto tmpu = T(m_cache.local().quad.points()(i, 0));
                auto tmpv = T(m_cache.local().quad.points()(i, 1));
                if (!check_degenerate())
                    continue;
                else {
                    Eigen::Matrix<T, 3, 1> tmpp_displaced = get(tmpu, tmpv);
                    Eigen::Matrix<T, 3, 1> tmpp_tri = get_p_tri(tmpu, tmpv);
                    Eigen::Matrix<T, 3, 1> diffp = tmpp_displaced - tmpp_tri;
                    value += squared_norm_T(diffp) * T(m_cache.local().quad.weights()[i]);
                }
            }
        }
    }
    return value;
}

float sample_nearest(const wmtk::Image& image, float u, float v)
{
    auto w = image.width();
    auto h = image.height();
    // x, y are between 0 and 1
    auto x = u * static_cast<float>(w);
    auto y = v * static_cast<float>(h);

    const auto sx = std::clamp(static_cast<int>(x), 0, w - 1);
    const auto sy = std::clamp(static_cast<int>(y), 0, h - 1);

    const auto& array = image.get_raw_image();

    return array(sx, sy);
}

float sample_bilinear(const wmtk::Image& image, float u, float v)
{
    auto w = image.width();
    auto h = image.height();
    // x, y are between 0 and 1
    auto x = u * static_cast<float>(w);
    auto y = v * static_cast<float>(h);

    auto sample_coord = [](const float coord,
                           const size_t size) -> std::tuple<size_t, size_t, float> {
        assert(_0 <= coord && coord <= static_cast<float>(size));
        size_t coord0, coord1;
        float t;
        if (coord <= 0.5f) {
            coord0 = 0;
            coord1 = 0;
            t = 0.5f + coord;
        } else if (coord + 0.5f >= static_cast<float>(size)) {
            coord0 = size - 1;
            coord1 = size - 1;
            t = coord - (static_cast<float>(coord0) + 0.5f);
        } else {
            assert(1 < size);
            coord0 = std::min(size - 2, static_cast<size_t>(coord - 0.5f));
            coord1 = coord0 + 1;
            t = coord - (static_cast<float>(coord0) + 0.5f);
        }
        return std::make_tuple(coord0, coord1, t);
    };

    const auto [x0, x1, tx] = sample_coord(x, w);
    const auto [y0, y1, ty] = sample_coord(y, h);

    const auto& array = image.get_raw_image();

    Eigen::Vector4f pix(array(x0, y0), array(x1, y0), array(x0, y1), array(x1, y1));
    Eigen::Vector4f weight((1.f - tx) * (1.f - ty), tx * (1.f - ty), (1.f - tx) * ty, tx * ty);

    return pix.dot(weight);
}


} // namespace

struct TextureIntegral::Cache
{
public:
    Cache(std::array<wmtk::SamplingBicubic, 3> samplers_)
        : samplers(std::move(samplers_))
    {}

public:
    // Data for exact error computation
    tbb::enumerable_thread_specific<QuadratureCache> quadrature_cache;
    std::array<wmtk::SamplingBicubic, 3> samplers;
};

TextureIntegral::TextureIntegral(std::array<wmtk::Image, 3> data)
    : m_data(std::move(data))
    , m_cache(lagrange::make_value_ptr<Cache>(std::array<wmtk::SamplingBicubic, 3>{
          SamplingBicubic(m_data[0]),
          SamplingBicubic(m_data[1]),
          SamplingBicubic(m_data[2])}))
{}

TextureIntegral::~TextureIntegral() = default;

void TextureIntegral::get_error_per_triangle(
    lagrange::span<const std::array<float, 6>> input_triangles,
    lagrange::span<float> output_errors,
    int flag)
{
    assert(input_triangles.size() == output_errors.size());
    if (flag == 0) {
        tbb::parallel_for(size_t(0), input_triangles.size(), [&](size_t i) {
            Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle;
            triangle.row(0) << input_triangles[i][0], input_triangles[i][1];
            triangle.row(1) << input_triangles[i][2], input_triangles[i][3];
            triangle.row(2) << input_triangles[i][4], input_triangles[i][5];
            output_errors[i] = get_error_per_triangle_exact(
                m_data[0],
                triangle,
                m_cache->quadrature_cache,
                [&](double u, double v) -> Eigen::Matrix<double, 3, 1> {
                    Eigen::Matrix<double, 3, 1> displaced_position;
                    for (auto i = 0; i < 3; ++i) {
                        displaced_position[i] = m_cache->samplers[i].sample(u, v);
                    }
                    return displaced_position;
                });
        });
    } else if (flag == 1) {
        tbb::parallel_for(size_t(0), input_triangles.size(), [&](size_t i) {
            Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle;
            triangle.row(0) << input_triangles[i][0], input_triangles[i][1];
            triangle.row(1) << input_triangles[i][2], input_triangles[i][3];
            triangle.row(2) << input_triangles[i][4], input_triangles[i][5];
            output_errors[i] = get_error_per_triangle_exact(
                m_data[0],
                triangle,
                m_cache->quadrature_cache,
                [&](double u, double v) -> Eigen::Matrix<double, 3, 1> {
                    Eigen::Matrix<double, 3, 1> displaced_position;
                    for (auto i = 0; i < 3; ++i) {
                        displaced_position[i] = sample_nearest(m_data[i], u, v);
                    }
                    return displaced_position;
                });
        });
    } else if (flag == 2) {
        tbb::parallel_for(size_t(0), input_triangles.size(), [&](size_t i) {
            Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle;
            triangle.row(0) << input_triangles[i][0], input_triangles[i][1];
            triangle.row(1) << input_triangles[i][2], input_triangles[i][3];
            triangle.row(2) << input_triangles[i][4], input_triangles[i][5];
            output_errors[i] = get_error_per_triangle_exact(
                m_data[0],
                triangle,
                m_cache->quadrature_cache,
                [&](double u, double v) -> Eigen::Matrix<double, 3, 1> {
                    Eigen::Matrix<double, 3, 1> displaced_position;
                    for (auto i = 0; i < 3; ++i) {
                        displaced_position[i] = sample_bilinear(m_data[i], u, v);
                    }
                    return displaced_position;
                });
        });
    }
}

void TextureIntegral::get_integral_per_triangle(
    lagrange::span<const std::array<float, 6>> input_triangles,
    lagrange::span<std::array<float, 3>> output_integrals)
{}

} // namespace wmtk
