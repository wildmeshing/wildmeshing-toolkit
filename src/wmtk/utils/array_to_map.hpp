#pragma once

namespace wmtk::utils {

template <typename T, int R, int C>
auto array_to_map(const T (&a)[R][C])
{
    return typename Eigen::Matrix<T, R, C>::ConstMapType(a);
}
template <typename T, int R, int C>
auto array_to_map(T (&a)[R][C])
{
    return typename Eigen::Matrix<T, R, C>::MapType(a);
}
template <typename T, int R>
auto array_to_map(const T (&a)[R])
{
    return typename Eigen::Matrix<T, R, 1>::ConstMapType(a);
}
template <typename T, int R>
auto array_to_map(T (&a)[R])
{
    return typename Eigen::Matrix<T, R, 1>::MapType(a);
}
} // namespace wmtk::utils
