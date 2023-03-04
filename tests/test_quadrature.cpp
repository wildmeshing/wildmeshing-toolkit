#include <wmtk/quadrature/ClippedQuadrature.h>
#include <wmtk/quadrature/TriangleQuadrature.h>
#include <wmtk/utils/PolygonClipping.h>
#include <wmtk/utils/autodiff.h>

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
double eval_over_box(const Eigen::AlignedBox2d& box, Func f)
{
    return f(box.max().x(), box.max().y()) - f(box.min().x(), box.max().y()) -
           f(box.max().x(), box.min().y()) + f(box.min().x(), box.min().y());
}

Eigen::AlignedBox2d triangle_bbox(const TriangleVertices& triangle)
{
    Eigen::AlignedBox2d box;
    for (auto p : triangle.rowwise()) {
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

        auto box = triangle_bbox(triangle0);
        auto expected = eval_over_box(box, f);
        auto value = integrate(order, triangle0, g) + integrate(order, triangle1, g);
        CAPTURE(Degree, type);
        REQUIRE_THAT(value, Catch::Matchers::WithinRel(expected, 1e-14));
    }
}

} // namespace

TEST_CASE("test_triangle_polygon_quadrature", "[quadrature]")
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
