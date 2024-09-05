#pragma once
namespace wmtk {
class PointMesh;
class EdgeMesh;
class TriMesh;
class TetMesh;
} // namespace wmtk
namespace wmtk::utils::metaprogramming {

namespace detail {
template <typename MeshType>
struct cell_dimension
{
};
template <>
struct cell_dimension<PointMesh>
{
    constexpr static int64_t value = 0;
};
template <>
struct cell_dimension<EdgeMesh>
{
    constexpr static int64_t value = 1;
};
template <>
struct cell_dimension<TriMesh>
{
    constexpr static int64_t value = 2;
};
template <>
struct cell_dimension<TetMesh>
{
    constexpr static int64_t value = 3;
};
} // namespace detail
template <typename MeshType>
constexpr static int64_t cell_dimension_v = detail::cell_dimension<MeshType>::value;

/*
constexpr static int64_t cell_dimension_v<PointMesh> = 0;
constexpr static int64_t cell_dimension_v<EdgeMesh> = 1;
constexpr static int64_t cell_dimension_v<TriMesh> = 2;
constexpr static int64_t cell_dimension_v<EdgeMesh> = 3;
*/


} // namespace wmtk::utils::metaprogramming
