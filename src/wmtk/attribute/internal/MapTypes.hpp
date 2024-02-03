#pragma once
#include <Eigen/Core>

namespace wmtk::attribute::internal {

    // this value is set by CMake
    constexpr static int MAX_ATTR_SIZE = WMTK_MAX_ATTRIBUTE_DIMENSION;
    template <typename T,int R = Eigen::Dynamic>
    using MapResult =
        Eigen::Map<Eigen::Matrix<T, R, 1, 0, (R == Eigen::Dynamic ? MAX_ATTR_SIZE : R), 1>>;
    template <typename T, int R = Eigen::Dynamic>
    using ConstMapResult =
        Eigen::Map<const Eigen::Matrix<T, R, 1, 0, (R == Eigen::Dynamic ? MAX_ATTR_SIZE : R), 1>>;

}
