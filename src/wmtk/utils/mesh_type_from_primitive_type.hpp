#pragma once

#include <wmtk/Primitive.hpp>

namespace wmtk {
class Mesh;
class EdgeMesh;
class PointMesh;
class TetMesh;
class TriMesh;
} // namespace wmtk
namespace wmtk::utils {

// helper functions for identifying a simplicial complex from a primtivie type
template <PrimitiveType pt>
struct mesh_type_from_primitive_type
{
    // if we ever call this something is wrong, but leaving this impl to let compilation work
    using type = Mesh;
};
template <PrimitiveType pt>
using mesh_type_from_primitive_type_t = typename mesh_type_from_primitive_type<pt>::type;

template <>
struct mesh_type_from_primitive_type<PrimitiveType::Vertex>
{
    using type = PointMesh;
};
template <>
struct mesh_type_from_primitive_type<PrimitiveType::Edge>
{
    using type = EdgeMesh;
};
template <>
struct mesh_type_from_primitive_type<PrimitiveType::Triangle>
{
    using type = TriMesh;
};

template <>
struct mesh_type_from_primitive_type<PrimitiveType::Tetrahedron>
{
    using type = TetMesh;
};

template <int8_t DIM>
struct mesh_type_from_dimension
{
    constexpr static PrimitiveType primitive_type = get_primitive_type_from_id(DIM);
    using type = wmtk::utils::mesh_type_from_primitive_type_t<primitive_type>;
};


template <int8_t DIM>
using mesh_type_from_dimension_t = typename mesh_type_from_dimension<DIM>::type;
} // namespace wmtk::utils
