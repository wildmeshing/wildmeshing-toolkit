#include "Eigen.h"
#include <spdlog/fmt/bundled/format.h>


namespace {
template <typename T, int R>
HighFive::DataType create()
{
    static_assert(R > 1);
    using MD = HighFive::CompoundType::member_def;
    std::vector<MD> ret;
    HighFive::DataType ttype = HighFive::create_datatype<T>();
    for (int j = 0; j < R; ++j) {
        ret.emplace_back(fmt::format("v{}", j), ttype);
    }
    return HighFive::CompoundType(ret);
}
} // namespace

template <>
HighFive::DataType HighFive::create_datatype<Eigen::Vector4d>()
{
    return create<double, 4>();
}
template <>
HighFive::DataType HighFive::create_datatype<Eigen::Vector3d>()
{
    return create<double, 3>();
}
template <>
HighFive::DataType HighFive::create_datatype<Eigen::Vector2d>()
{
    return create<double, 2>();
}

template <>
HighFive::DataType HighFive::create_datatype<Eigen::Vector4f>()
{
    return create<float, 4>();
}
template <>
HighFive::DataType HighFive::create_datatype<Eigen::Vector3f>()
{
    return create<float, 3>();
}
template <>
HighFive::DataType HighFive::create_datatype<Eigen::Vector2f>()
{
    return create<float, 2>();
}


template <>
HighFive::DataType HighFive::create_datatype<Eigen::Vector4i>()
{
    return create<int, 4>();
}
template <>
HighFive::DataType HighFive::create_datatype<Eigen::Vector3i>()
{
    return create<int, 3>();
}
template <>
HighFive::DataType HighFive::create_datatype<Eigen::Vector2i>()
{
    return create<int, 2>();
}
