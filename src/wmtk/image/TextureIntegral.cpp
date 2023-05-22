#include "TextureIntegral.h"

#include "helpers.h"

#include <wmtk/quadrature/ClippedQuadrature.h>

#include <lagrange/utils/assert.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/parallel_for.h>

namespace wmtk {
namespace {

template <typename T, typename Func>
void sequential_for(T start, T end, Func func)
{
    for (T i = start; i < end; ++i) {
        func(i);
    }
}

struct QuadratureCache
{
    wmtk::Quadrature quad;
    wmtk::Quadrature tmp;
};

// Reference implementation copied over from Displacement.h
template <typename T, typename DisplacementFunc>
T get_error_per_triangle_exact(
    const wmtk::Image& m_image,
    const Eigen::Matrix<T, 3, 2, Eigen::RowMajor>& triangle,
    tbb::enumerable_thread_specific<QuadratureCache>& cache,
    DisplacementFunc get)
{
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
                cache.local().quad,
                &cache.local().tmp);
            for (auto i = 0; i < cache.local().quad.size(); ++i) {
                auto tmpu = T(cache.local().quad.points()(i, 0));
                auto tmpv = T(cache.local().quad.points()(i, 1));
                if (!check_degenerate())
                    continue;
                else {
                    Eigen::Matrix<T, 3, 1> tmpp_displaced = get(tmpu, tmpv);
                    Eigen::Matrix<T, 3, 1> tmpp_tri = get_p_tri(tmpu, tmpv);
                    Eigen::Matrix<T, 3, 1> diffp = tmpp_displaced - tmpp_tri;
                    value += squared_norm_T(diffp) * T(cache.local().quad.weights()[i]);
                }
            }
        }
    }
    return value;
}

// Optimized implementation that switches between nearest and bilinear interpolation
template <typename T, typename DisplacementFuncT, typename DisplacementFuncS>
T get_error_per_triangle_adaptive(
    const std::array<wmtk::Image, 3>& images,
    const Eigen::Matrix<T, 3, 2, Eigen::RowMajor>& triangle_uv,
    tbb::enumerable_thread_specific<QuadratureCache>& cache,
    const int order,
    DisplacementFuncT get_diff,
    DisplacementFuncS get_scalar)
{
    Eigen::Matrix3<T> triangle_3d;
    triangle_3d.col(0) = get_diff(triangle_uv(0, 0), triangle_uv(0, 1));
    triangle_3d.col(1) = get_diff(triangle_uv(1, 0), triangle_uv(1, 1));
    triangle_3d.col(2) = get_diff(triangle_uv(2, 0), triangle_uv(2, 1));

    Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle_uv_d;
    std::transform(
        triangle_uv.data(),
        triangle_uv.data() + triangle_uv.size(),
        triangle_uv_d.data(),
        [](auto x) { return get_value(x); });

    Eigen::AlignedBox2d bbox_uv;
    for (const auto& p : triangle_uv_d.rowwise()) {
        bbox_uv.extend(p.transpose());
    }

    const int w = images[0].width();
    const int h = images[0].height();
    auto get_coordinate = [&](double u, double v) -> Eigen::Vector2i {
        auto x = u * static_cast<float>(w);
        auto y = v * static_cast<float>(h);
        const auto sx = std::clamp(static_cast<int>(x), 0, w - 1);
        const auto sy = std::clamp(static_cast<int>(y), 0, h - 1);
        return {sx, sy};
    };

    const auto min_pixel = get_coordinate(bbox_uv.min()(0), bbox_uv.min()(1));
    const auto max_pixel = get_coordinate(bbox_uv.max()(0), bbox_uv.max()(1));
    const Eigen::Vector2d pixel_size(1.0 / w, 1.0 / h);

    const auto u1 = triangle_uv(0, 0);
    const auto v1 = triangle_uv(0, 1);
    const auto u2 = triangle_uv(1, 0);
    const auto v2 = triangle_uv(1, 1);
    const auto u3 = triangle_uv(2, 0);
    const auto v3 = triangle_uv(2, 1);
    const auto denom = ((v2 - v3) * (u1 - u3) + (u3 - u2) * (v1 - v3));
    if (get_value(denom) < std::numeric_limits<double>::denorm_min()) {
        // Degenerate triangle
        return T(0.0);
    }

    const std::array<Eigen::Hyperplane<double, 2>, 3> edges = {
        Eigen::Hyperplane<double, 2>::Through(triangle_uv_d.row(0), triangle_uv_d.row(1)),
        Eigen::Hyperplane<double, 2>::Through(triangle_uv_d.row(1), triangle_uv_d.row(2)),
        Eigen::Hyperplane<double, 2>::Through(triangle_uv_d.row(2), triangle_uv_d.row(0)),
    };

    // calculate the barycentric coordinate of the a point using u, v coordinates
    // returns the 3d coordinate on the current mesh
    auto get_p_interpolated = [&](double u, double v) -> Eigen::Matrix<T, 3, 1> {
        /*
        λ1 = ((v2 - v3)(u - u3) + (u3 - u2)(v - v3)) / ((v2 - v3)(u1 - u3) + (u3 - u2)(v1 - v3))
        λ2 = ((v3 - v1)(u - u3) + (u1 - u3)(v - v3)) / ((v2 - v3)(u1 - u3) + (u3 - u2)(v1 - v3))
        λ3 = 1 - λ1 - λ2
        z = λ1 * z1 + λ2 * z2 + λ3 * z3
        */
        auto lambda1 = ((v2 - v3) * (u - u3) + (u3 - u2) * (v - v3)) / denom;
        auto lambda2 = ((v3 - v1) * (u - u3) + (u1 - u3) * (v - v3)) / denom;
        auto lambda3 = 1 - lambda1 - lambda2;
        return lambda1 * triangle_3d.col(0) + lambda2 * triangle_3d.col(1) +
               lambda3 * triangle_3d.col(2);
    };

    T value = T(0.);
    size_t num_inside = 0;
    size_t num_total = 0;
    size_t num_boundary = 0;
    const double pixel_radius = pixel_size.norm() * 0.5;
    for (int y = min_pixel.y(); y <= max_pixel.y(); ++y) {
        for (int x = min_pixel.x(); x <= max_pixel.x(); ++x) {
            ++num_total;
            Eigen::Vector2i pixel_coord(x, y);
            Eigen::AlignedBox2d box;
            box.extend(pixel_coord.cast<double>().cwiseProduct(pixel_size));
            box.extend(
                (pixel_coord + Eigen::Vector2i::Ones()).cast<double>().cwiseProduct(pixel_size));
            auto sign = internal::point_in_triangle_quick(edges, box.center(), pixel_radius);
            if (sign == internal::Classification::Unknown) {
                sign = internal::pixel_inside_triangle(triangle_uv_d, box);
            }
            if (sign == internal::Classification::Outside) {
                continue;
            }
            if (sign == internal::Classification::Inside) {
                ++num_inside;
                Eigen::Matrix<T, 3, 1> p_tri =
                    get_p_interpolated(box.center().x(), box.center().y());
                Eigen::Matrix<T, 3, 1> p_displaced = internal::fetch_texels(images, x, y).cast<T>();
                value += (p_displaced - p_tri).squaredNorm() * pixel_size(0) * pixel_size(1);
                if (0) {
                    // For debugging only
                    if constexpr (std::is_same_v<T, double>) {
                        wmtk::ClippedQuadrature rules;
                        auto& quadr = cache.local().quad;
                        rules.clipped_triangle_box_quadrature(
                            1,
                            triangle_uv_d,
                            box,
                            quadr,
                            &cache.local().tmp);
                        la_runtime_assert(quadr.size() == 2);
                        for (size_t i = 0; i < quadr.size(); ++i) {
                            double u = quadr.points()(i, 0);
                            double v = quadr.points()(i, 1);
                            Eigen::Matrix<T, 3, 1> p_displaced2 =
                                internal::sample_nearest(images, u, v).cast<T>();
                            Eigen::Matrix<T, 3, 1> p_tri2 = get_p_interpolated(u, v);
                            la_runtime_assert(p_displaced2 == p_displaced);
                        }
                        double diff =
                            std::abs(quadr.weights().sum() - pixel_size(0) * pixel_size(1));
                        la_runtime_assert(diff < 1e-10);
                    }
                }
            } else {
                wmtk::ClippedQuadrature rules;
                auto& quadr = cache.local().quad;
                rules.clipped_triangle_box_quadrature(
                    order,
                    triangle_uv_d,
                    box,
                    quadr,
                    &cache.local().tmp);
                if (quadr.size() > 0) {
                    ++num_boundary;
                }
                for (size_t i = 0; i < quadr.size(); ++i) {
                    const double u = quadr.points()(i, 0);
                    const double v = quadr.points()(i, 1);
                    Eigen::Matrix<T, 3, 1> p_displaced = get_scalar(u, v).template cast<T>();
                    Eigen::Matrix<T, 3, 1> p_tri = get_p_interpolated(u, v);
                    value += (p_displaced - p_tri).squaredNorm() * quadr.weights()[i];
                }
            }
        }
    }
    // logger().info("Num inside: {}, boundary: {}, total: {}", num_inside, num_boundary,
    // num_total);
    return value;
}

} // namespace

struct TextureIntegral::Cache
{
    // Data for exact error computation
    mutable tbb::enumerable_thread_specific<QuadratureCache> quadrature_cache;
};

TextureIntegral::TextureIntegral() = default;
TextureIntegral::TextureIntegral(TextureIntegral&&) = default; // move constructor
TextureIntegral& TextureIntegral::operator=(TextureIntegral&&) =
    default; // move assignment operator

TextureIntegral::TextureIntegral(std::array<wmtk::Image, 3> data)
    : m_data(std::move(data))
    , m_cache(lagrange::make_value_ptr<Cache>())
{}

TextureIntegral::~TextureIntegral() = default;

////////////////////////////////////////////////////////////////////////////////

template <TextureIntegral::SamplingMethod Sampling, TextureIntegral::IntegrationMethod Integration>
void TextureIntegral::get_error_per_triangle_internal(
    lagrange::span<const std::array<float, 6>> input_triangles,
    lagrange::span<float> output_errors,
    int order) const
{
    assert(input_triangles.size() == output_errors.size());
    tbb::parallel_for(size_t(0), input_triangles.size(), [&](size_t i) {
        Eigen::Matrix<double, 3, 2, Eigen::RowMajor> triangle;
        triangle.row(0) << input_triangles[i][0], input_triangles[i][1];
        triangle.row(1) << input_triangles[i][2], input_triangles[i][3];
        triangle.row(2) << input_triangles[i][4], input_triangles[i][5];
        auto sampling_func = [&](double u, double v) -> Eigen::Matrix<double, 3, 1> {
            if constexpr (Sampling == SamplingMethod::Bicubic) {
                return internal::sample_bicubic(m_data, u, v);
            } else if constexpr (Sampling == SamplingMethod::Nearest) {
                return internal::sample_nearest(m_data, u, v);
            } else if constexpr (Sampling == SamplingMethod::Bilinear) {
                return internal::sample_bilinear(m_data, u, v);
            }
        };
        if constexpr (Integration == IntegrationMethod::Exact) {
            output_errors[i] = get_error_per_triangle_exact(
                m_data[0],
                triangle,
                m_cache->quadrature_cache,
                sampling_func);
        } else if constexpr (Integration == IntegrationMethod::Adaptive) {
            output_errors[i] = get_error_per_triangle_adaptive(
                m_data,
                triangle,
                m_cache->quadrature_cache,
                order,
                sampling_func,
                sampling_func);
        }
    });
}

void TextureIntegral::get_error_per_triangle(
    lagrange::span<const std::array<float, 6>> input_triangles,
    lagrange::span<float> output_errors) const
{
    int order = 1;
    if (m_quadrature_order == QuadratureOrder::Full) {
        constexpr int Degree = 2;
        order = 2 * (Degree - 1);
    }

    assert(input_triangles.size() == output_errors.size());
    switch (m_sampling_method) {
    case SamplingMethod::Bicubic:
        if (m_integration_method == IntegrationMethod::Exact) {
            get_error_per_triangle_internal<SamplingMethod::Bicubic, IntegrationMethod::Exact>(
                input_triangles,
                output_errors,
                order);

        } else {
            get_error_per_triangle_internal<SamplingMethod::Bicubic, IntegrationMethod::Adaptive>(
                input_triangles,
                output_errors,
                order);
        }
        break;
    case SamplingMethod::Nearest:
        if (m_integration_method == IntegrationMethod::Exact) {
            get_error_per_triangle_internal<SamplingMethod::Nearest, IntegrationMethod::Exact>(
                input_triangles,
                output_errors,
                order);

        } else {
            get_error_per_triangle_internal<SamplingMethod::Nearest, IntegrationMethod::Adaptive>(
                input_triangles,
                output_errors,
                order);
        }
        break;
    case SamplingMethod::Bilinear:
        if (m_integration_method == IntegrationMethod::Exact) {
            get_error_per_triangle_internal<SamplingMethod::Bilinear, IntegrationMethod::Exact>(
                input_triangles,
                output_errors,
                order);

        } else {
            get_error_per_triangle_internal<SamplingMethod::Bilinear, IntegrationMethod::Adaptive>(
                input_triangles,
                output_errors,
                order);
        }
        break;
    }
}

////////////////////////////////////////////////////////////////////////////////

template <TextureIntegral::SamplingMethod Sampling, TextureIntegral::IntegrationMethod Integration>
auto TextureIntegral::get_error_one_triangle_internal(const DTriangle& input_triangle, int order)
    const -> DScalar
{
    auto sampling_diff = [&](DScalar u, DScalar v) -> Eigen::Matrix<DScalar, 3, 1> {
        if constexpr (Sampling == SamplingMethod::Bicubic) {
            return internal::sample_bicubic(m_data, u, v);
        } else if constexpr (Sampling == SamplingMethod::Nearest) {
            return internal::sample_nearest(m_data, u, v);
        } else if constexpr (Sampling == SamplingMethod::Bilinear) {
            return internal::sample_bilinear(m_data, u, v);
        }
    };
    auto sampling_scalar = [&](double u, double v) -> Eigen::Matrix<double, 3, 1> {
        if constexpr (Sampling == SamplingMethod::Bicubic) {
            return internal::sample_bicubic(m_data, u, v);
        } else if constexpr (Sampling == SamplingMethod::Nearest) {
            return internal::sample_nearest(m_data, u, v);
        } else if constexpr (Sampling == SamplingMethod::Bilinear) {
            return internal::sample_bilinear(m_data, u, v);
        }
    };
    if constexpr (Integration == IntegrationMethod::Exact) {
        return get_error_per_triangle_exact(
            m_data[0],
            input_triangle,
            m_cache->quadrature_cache,
            sampling_diff);
    } else if constexpr (Integration == IntegrationMethod::Adaptive) {
        return get_error_per_triangle_adaptive(
            m_data,
            input_triangle,
            m_cache->quadrature_cache,
            order,
            sampling_diff,
            sampling_scalar);
    }
}

auto TextureIntegral::get_error_one_triangle(const DTriangle& input_triangle) const -> DScalar
{
    int order = 1;
    if (m_quadrature_order == QuadratureOrder::Full) {
        constexpr int Degree = 2;
        order = 2 * (Degree - 1);
    }

    switch (m_sampling_method) {
    case SamplingMethod::Bicubic:
        if (m_integration_method == IntegrationMethod::Exact) {
            return get_error_one_triangle_internal<
                SamplingMethod::Bicubic,
                IntegrationMethod::Exact>(input_triangle, order);

        } else {
            return get_error_one_triangle_internal<
                SamplingMethod::Bicubic,
                IntegrationMethod::Adaptive>(input_triangle, order);
        }
    case SamplingMethod::Nearest:
        if (m_integration_method == IntegrationMethod::Exact) {
            return get_error_one_triangle_internal<
                SamplingMethod::Nearest,
                IntegrationMethod::Exact>(input_triangle, order);

        } else {
            return get_error_one_triangle_internal<
                SamplingMethod::Nearest,
                IntegrationMethod::Adaptive>(input_triangle, order);
        }
    case SamplingMethod::Bilinear:
        if (m_integration_method == IntegrationMethod::Exact) {
            return get_error_one_triangle_internal<
                SamplingMethod::Bilinear,
                IntegrationMethod::Exact>(input_triangle, order);

        } else {
            return get_error_one_triangle_internal<
                SamplingMethod::Bilinear,
                IntegrationMethod::Adaptive>(input_triangle, order);
        }
    }
    throw std::runtime_error("Invalid sampling method or integration method");
}

} // namespace wmtk
