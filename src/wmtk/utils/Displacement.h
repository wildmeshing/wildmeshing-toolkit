#pragma once

#include <type_traits>
class Displacement
{
public:
    enum class WrappingMode { REPEAT, MIRROR_REPEAT, CLAMP_TO_EDGE };

    template <class T>
    std::decay_t<T> get(const T& u, const T& v) const;
    template <class T>
    std::decay_t<T> get_error_per_edge(
        const Eigen::Matrix<T, 2, 1>& p1,
        const Eigen::Matrix<T, 2, 1>& p2) const;
};
