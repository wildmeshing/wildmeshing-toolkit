
#include <catch2/catch_test_macros.hpp>

#include <wmtk/utils/autodiff.h>

#include <iostream>

namespace {

template <typename DScalar>
void test_autodiff_abs()
{
    // For custom numerical types, the default abs() function might be a no-op inside Eigen
    // matrices. The culprit is the following snippet in Eigen:
    //
    // https://gitlab.com/libeigen/eigen/-/blob/3.4/Eigen/src/Core/MathFunctions.h?ref_type=heads#L1514-1519
    //
    // One possible solution would be to specialize Eigen::NumTraits<> for DScalar:
    //
    // https://eigen.tuxfamily.org/dox/TopicCustomizing_CustomScalar.html
    //
    // However, I would argue that it is even better to specialize std::numeric_limits<> for DScalar
    // instead, since it is more general. Indeed, Eigen::GenericNumTraits<> will deduce its IsSigned
    // flag from std::numeric_limits<>:
    //
    // https://gitlab.com/libeigen/eigen/-/blob/3.4/Eigen/src/Core/NumTraits.h#L156
    //
    DScalar::setVariableCount(2);
    Eigen::Vector3<DScalar> x;
    x[0] = DScalar(-0.7731865662855606);
    x[1] = DScalar(-0.3609661432608684);
    x[2] = DScalar(-0.5214268665258044);

    x = x.stableNormalized();
    REQUIRE(x[0].getValue() < 0);
    REQUIRE(x[1].getValue() < 0);
    REQUIRE(x[2].getValue() < 0);
}

} // namespace

TEST_CASE("autodiff abs", "[autodiff]")
{
    test_autodiff_abs<DScalar1<double, Eigen::Vector2d>>();
    test_autodiff_abs<DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>>();
}
