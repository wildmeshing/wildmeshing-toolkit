#include <wmtk/quadrature/ClippedQuadrature.h>
#include <wmtk/quadrature/TriangleQuadrature.h>
#include <wmtk/utils/PolygonClipping.h>
#include <wmtk/utils/autodiff.h>
#include <wmtk/utils/Logger.hpp>

#include <catch2/catch.hpp>

#include <iostream>
#include <random>

namespace {

template <typename Scalar, int Degree>
class BivariatePolynomial
{
protected:
    static constexpr int NumCoeffs = Degree + 1;
    using MatrixNs = Eigen::Matrix<Scalar, NumCoeffs, NumCoeffs>;
    using VectorNs = Eigen::Vector<Scalar, NumCoeffs>;

public:
    static BivariatePolynomial random()
    {
        std::mt19937 gen;
        std::uniform_real_distribution<Scalar> dist(Scalar(-0.5f), Scalar(0.5f));

        BivariatePolynomial poly;
        poly.m_coeffs = MatrixNs::NullaryExpr([&]() { return dist(gen); });
        return poly;
    }

    static BivariatePolynomial identity()
    {
        BivariatePolynomial poly;
        poly.m_coeffs.setIdentity();
        return poly;
    }

    Scalar eval(Scalar x, Scalar y) const
    {
        VectorNs X;
        VectorNs Y;
        for (int i = 0; i < NumCoeffs; ++i) {
            X[i] = std::pow(x, i);
            Y[i] = std::pow(y, i);
        }
        return X.transpose() * m_coeffs * Y;
    }

    Scalar eval_dxdy(Scalar x, Scalar y) const
    {
        VectorNs X;
        VectorNs Y;
        for (int i = 0; i < NumCoeffs; ++i) {
            X[i] = (i == 0 ? Scalar(0) : Scalar(i) * std::pow(x, i - 1));
            Y[i] = (i == 0 ? Scalar(0) : Scalar(i) * std::pow(y, i - 1));
        }
        return X.transpose() * m_coeffs * Y;
    }

    const MatrixNs& coeffs() const { return m_coeffs; }

protected:
    Eigen::Matrix<Scalar, NumCoeffs, NumCoeffs> m_coeffs;
};

using TriangleVertices = wmtk::ClippedQuadrature::TriangleVertices;
using PolygonVertices = wmtk::ClippedQuadrature::PolygonVertices;

template <typename Func>
double integrate(int order, const TriangleVertices& triangle, Func f)
{
    wmtk::Quadrature quad;
    wmtk::TriangleQuadrature rules;
    rules.transformed_triangle_quadrature(order, triangle, quad);
    double result = 0;
    for (int i = 0; i < quad.size(); ++i) {
        result += f(quad.points(i, 0), quad.points(i, 1)) * quad.weights[i];
    }
    return result;
}

template <typename Func>
double integrate_reference(int order, Func f)
{
    wmtk::Quadrature quad;
    wmtk::TriangleQuadrature rules;
    rules.reference_triangle_quadrature(order, quad);
    double result = 0;
    for (int i = 0; i < quad.size(); ++i) {
        result += f(quad.points(i, 0), quad.points(i, 1)) * quad.weights[i];
    }
    return result;
}

template <typename Func>
double integrate_clipped_polygon(
    int order,
    const TriangleVertices& triangle,
    const PolygonVertices& poly,
    Func f)
{
    wmtk::Quadrature quad;
    wmtk::ClippedQuadrature rules;
    rules.clipped_triangle_polygon_quadrature(order, triangle, poly, quad);
    double result = 0;
    for (int i = 0; i < quad.size(); ++i) {
        result += f(quad.points(i, 0), quad.points(i, 1)) * quad.weights[i];
    }
    return result;
}

template <typename Func>
double integrate_clipped_box(
    int order,
    const TriangleVertices& triangle,
    const Eigen::AlignedBox2d& box,
    Func f)
{
    wmtk::Quadrature quad;
    wmtk::ClippedQuadrature rules;
    rules.clipped_triangle_box_quadrature(order, triangle, box, quad);
    double result = 0;
    for (int i = 0; i < quad.size(); ++i) {
        result += f(quad.points(i, 0), quad.points(i, 1)) * quad.weights[i];
    }
    return result;
}

template <typename Func>
double eval_over_box(const Eigen::AlignedBox2d& box, Func f)
{
    return f(box.max().x(), box.max().y()) - f(box.min().x(), box.max().y()) -
           f(box.max().x(), box.min().y()) + f(box.min().x(), box.min().y());
}

template<typename Derived>
Eigen::AlignedBox2d points_bbox(const Eigen::MatrixBase<Derived>& pts)
{
    Eigen::AlignedBox2d box;
    for (auto p : pts.rowwise()) {
        box.extend(p.transpose());
    }
    return box;
}

enum PolynomialType {
    Random = 0,
    Identity = 1,
};

void test_reference_quadrature()
{
    constexpr int Degree = 3;
    using MyBivariatePolynomial = BivariatePolynomial<double, Degree>;

    // Create a bivariate polynomial
    auto poly = MyBivariatePolynomial::identity();

    auto g = [&](double x, double y) { return poly.eval_dxdy(x, y); };

    // Integral over reference triangle. Computed via wolfram alpha:
    //
    // Integrate[\(91)0\(44) 1\(44) 2*x\(44) 3*Power[x,2]\(93) . \(91)0\(44) 1\(44) 2*y\(44)
    // 3*Power[y,2]\(93),{x,0,1-y},{y,0,1}]
    //
    double expected = 43.0 / 60.0;
    for (int order = 2 * (Degree - 1); order <= 15; ++order) {
        auto value = integrate_reference(order, g);
        CAPTURE(order);
        REQUIRE_THAT(value, Catch::Matchers::WithinRel(expected, 1e-14));
    }
}

template <int Degree, PolynomialType type>
void test_triangle_quadrature()
{
    using MyBivariatePolynomial = BivariatePolynomial<double, Degree>;
    const int order = 2 * (Degree - 1);

    // Create a bivariate polynomial
    auto poly = [] {
        if (type == PolynomialType::Random) {
            return MyBivariatePolynomial::random();
        } else {
            return MyBivariatePolynomial::identity();
        }
    }();

    auto f = [&](double x, double y) { return poly.eval(x, y); };
    auto g = [&](double x, double y) { return poly.eval_dxdy(x, y); };

    // Integral over two triangles
    {
        TriangleVertices triangle0;
        TriangleVertices triangle1;
        triangle0 << 0, 0, 1, 0, 0, 1;
        triangle1 << 1, 1, 0, 1, 1, 0;

        auto box = points_bbox(triangle0);
        auto expected = eval_over_box(box, f);
        auto value = integrate(order, triangle0, g) + integrate(order, triangle1, g);
        CAPTURE(Degree, type);
        REQUIRE_THAT(value, Catch::Matchers::WithinRel(expected, 1e-14));
    }
}

std::string format_poly(const Eigen::MatrixXd& pts)
{
    std::vector<std::string> items;
    for (const auto& p : pts.rowwise()) {
        items.push_back(fmt::format("{:.2f},{:.2f}", p.x(), p.y()));
    }
    return fmt::format("[{}] ({} pts)", fmt::join(items, " ; "), pts.rows());
}

void save_poly(const std::string &filename, const PolygonVertices &pts) {
    int num_vertices = pts.rows();
    assert(num_vertices >= 3);
    Eigen::MatrixXd V(num_vertices, 3);
    V.leftCols<2>() = pts;
    V.col(2).setZero();
    Eigen::MatrixXi F(num_vertices - 2, 3);
    for (int i = 1; i + 1 < num_vertices; ++i) {
        F.row(i - 1) << 0, i, i+1;
    }
    (void) filename;
    // igl::write_triangle_mesh(filename, V, F);
}

std::vector<PolygonVertices> split_polygon_randomly(const PolygonVertices& contour, int num)
{
    const auto bbox = points_bbox(contour);
    const double diag = bbox.diagonal().norm();

    std::mt19937 gen;
    std::uniform_real_distribution<double> angle(0.0, M_PI);
    std::normal_distribution<double> offset(0.0, 0.3 * diag);

    std::vector<PolygonVertices> current = {contour};
    std::vector<PolygonVertices> next;

    for (int i = 0; i < num; ++i) {
        next.clear();
        const double theta = angle(gen);
        const double lambda = offset(gen);
        const Eigen::RowVector2d nrm(-std::sin(theta), std::cos(theta));
        const Eigen::RowVector2d q1 = bbox.center().transpose() + lambda * nrm;
        const Eigen::RowVector2d q2 = q1 + Eigen::RowVector2d(std::cos(theta), std::sin(theta));
        wmtk::logger().debug(
            "- Splitting angle is: {:.2f}, offset is {:.2f}, q1=({:.2f}, {:.2f}), q2=({:.2f}, {:.2f})",
            theta,
            lambda,
            q1.x(),
            q1.y(),
            q2.x(),
            q2.y());
        for (const auto& poly : current) {
            Eigen::MatrixXd pts = poly;
            Eigen::MatrixXd result;
            wmtk::logger().debug("- Splitting poly: {}", format_poly(pts));
            wmtk::clip_polygon_by_half_plane(pts, q1, q2, result);
            if (result.rows() > 0) {
                wmtk::logger().debug("\t- Left poly: {}", format_poly(result));
                next.push_back(result);
            }
            wmtk::clip_polygon_by_half_plane(pts, q2, q1, result);
            if (result.rows() > 0) {
                wmtk::logger().debug("\t- Right poly: {}", format_poly(result));
                next.push_back(result);
            }
        }
        std::swap(current, next);
    }
    wmtk::logger().info("Split poly {} times in {} components", num, current.size());

    for (size_t i = 0; i < current.size(); ++i) {
        save_poly(fmt::format("poly_{}.obj", i), current[i]);
    }

    return current;
}

std::vector<PolygonVertices> split_bbox_randomly(const Eigen::AlignedBox2d& box, int num)
{
    PolygonVertices poly(4, 2);
    poly.row(0) = box.corner(Eigen::AlignedBox2d::CornerType::BottomLeft).transpose();
    poly.row(1) = box.corner(Eigen::AlignedBox2d::CornerType::BottomRight).transpose();
    poly.row(2) = box.corner(Eigen::AlignedBox2d::CornerType::TopRight).transpose();
    poly.row(3) = box.corner(Eigen::AlignedBox2d::CornerType::TopLeft).transpose();
    return split_polygon_randomly(poly, num);
}

template <int Degree>
void test_clipped_quadrature()
{
    using MyBivariatePolynomial = BivariatePolynomial<double, Degree>;
    const int order = 2 * (Degree - 1);

    // Create a bivariate polynomial
    auto poly = MyBivariatePolynomial::random();

    auto g = [&](double x, double y) { return poly.eval_dxdy(x, y); };

    // Integral over one triangle
    {
        TriangleVertices triangle;
        triangle << 0, 0.2, 1, 0.5, 0.3, 1;

        auto box = points_bbox(triangle);
        auto expected = integrate(order, triangle, g);
        auto polys = split_bbox_randomly(box, 6);
        double value = 0;
        for (const auto& p : polys) {
            value += integrate_clipped_polygon(order, triangle, p, g);
        }
        CAPTURE(Degree);
        REQUIRE_THAT(value, Catch::Matchers::WithinRel(expected, 1e-14));
    }
}

} // namespace

TEST_CASE("test_triangle_quadrature", "[quadrature]")
{
    test_reference_quadrature();

    test_triangle_quadrature<2, PolynomialType::Identity>();
    test_triangle_quadrature<3, PolynomialType::Identity>();
    test_triangle_quadrature<4, PolynomialType::Identity>();
    test_triangle_quadrature<5, PolynomialType::Identity>();

    test_triangle_quadrature<2, PolynomialType::Random>();
    test_triangle_quadrature<3, PolynomialType::Random>();
    test_triangle_quadrature<4, PolynomialType::Random>();
    test_triangle_quadrature<5, PolynomialType::Random>();
}

TEST_CASE("test_clipped_quadrature", "[quadrature]")
{
    test_clipped_quadrature<2>();
    test_clipped_quadrature<3>();
    test_clipped_quadrature<4>();
    test_clipped_quadrature<5>();
}
