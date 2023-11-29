
#include <wmtk/function/utils/autodiff.h>
#include <catch2/catch_test_macros.hpp>
#include <iostream>
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/function/utils/AutoDiffUtils.hpp>


TEST_CASE("analytic_autodiff", "[autodiff]")
{
    auto raii = wmtk::function::utils::AutoDiffRAII(2);
    REQUIRE(DiffScalarBase::getVariableCount() == 2);
    Eigen::Vector2d x{2, 3};
    using DScalar = DScalar2<double, Eigen::Vector2d, Eigen::Matrix2d>;
    auto xD = wmtk::function::utils::as_DScalar<DScalar>(x);

    auto v = xD.x() * xD.y() * xD.y();

    CHECK(v.getValue() == 18);

    Eigen::Vector2d grad{9., 12.};
    CHECK(v.getGradient() == grad);
}
