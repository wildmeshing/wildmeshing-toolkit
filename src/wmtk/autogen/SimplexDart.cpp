#include "SimplexDart.hpp"
#include <cassert>
#include <wmtk/utils/TupleInspector.hpp>
#include "edge_mesh/SimplexDart.hpp"
#include "point_mesh/SimplexDart.hpp"
#include "subgroup/convert.hpp"
#include "tet_mesh/SimplexDart.hpp"
#include "tri_mesh/SimplexDart.hpp"
#include "tuple_from_valid_index.hpp"
#include "valid_index_from_tuple.hpp"

namespace wmtk::autogen {
namespace {


/*
int8_t empty_binary(int8_t,int8_t){
    assert(false);
}

int8_t empty_unary(int8_t){
    assert(false);
}
int8_t empty_unary(){
    assert(false);
}

template <typename T>
    void f();

template <typename
    */


#define GET_OP(NAME, RETTYPE)                                                 \
    auto get_##NAME(PrimitiveType pt) -> SimplexDart::RETTYPE                 \
    {                                                                         \
        switch (pt) {                                                         \
        case PrimitiveType::Edge: return &edge_mesh::SimplexDart::NAME;       \
        case PrimitiveType::Triangle: return &tri_mesh::SimplexDart::NAME;    \
        case PrimitiveType::Tetrahedron: return &tet_mesh::SimplexDart::NAME; \
        case PrimitiveType::Vertex: return &point_mesh::SimplexDart::NAME;    \
        default: assert(false);                                               \
        }                                                                     \
        return nullptr;                                                       \
    }
GET_OP(product, binary_op_type)
GET_OP(inverse, unary_op_type)
GET_OP(primitive_to_index, primitive_to_index_type)
GET_OP(identity, nullary_op_type)
} // namespace

#define FORWARD_OP(NAME, OP, RETTYPE, DEFAULT)                               \
    auto SimplexDart::NAME() const -> RETTYPE                                \
    {                                                                        \
        switch (m_simplex_type) {                                            \
        case PrimitiveType::Edge: return edge_mesh::SimplexDart::OP();       \
        case PrimitiveType::Triangle: return tri_mesh::SimplexDart::OP();    \
        case PrimitiveType::Tetrahedron: return tet_mesh::SimplexDart::OP(); \
        case PrimitiveType::Vertex: return point_mesh::SimplexDart::OP();    \
        default: assert(false);                                              \
        }                                                                    \
        return DEFAULT;                                                      \
    }

FORWARD_OP(size, size, size_t, {})
using DynamicIntMap = VectorX<int8_t>::ConstMapType;
namespace {
const static DynamicIntMap nullmap = DynamicIntMap(nullptr, 0);
}
FORWARD_OP(valid_indices, valid_indices_dynamic, DynamicIntMap, nullmap)

SimplexDart::SimplexDart(wmtk::PrimitiveType simplex_type)
    : m_simplex_type(simplex_type)
    , m_product(get_product(simplex_type))
    , m_inverse(get_inverse(simplex_type))
    , m_primitive_to_index(get_primitive_to_index(simplex_type))
    , m_identity(get_identity(simplex_type))
{}

int8_t SimplexDart::product(int8_t a, int8_t b) const
{
    return m_product(a, b);
}
int8_t SimplexDart::inverse(int8_t a) const
{
    return m_inverse(a);
}
int8_t SimplexDart::primitive_as_index(wmtk::PrimitiveType pt) const
{
    return m_primitive_to_index(pt);
}
int8_t SimplexDart::identity() const
{
    return m_identity();
}
wmtk::Tuple SimplexDart::tuple_from_valid_index(int64_t gid, int8_t index) const
{
    return wmtk::autogen::tuple_from_valid_index(m_simplex_type, gid, index);
}
wmtk::Tuple SimplexDart::update_tuple_from_valid_index(const Tuple& t, int8_t index) const
{
    return wmtk::autogen::tuple_from_valid_index(
        m_simplex_type,
        wmtk::utils::TupleInspector::global_cid(t),
        index);
}
int8_t SimplexDart::valid_index_from_tuple(const wmtk::Tuple& t) const
{
    return wmtk::autogen::valid_index_from_tuple(m_simplex_type, t);
}

int8_t SimplexDart::convert(int8_t valid_index, const SimplexDart& target) const
{
    if (target.m_simplex_type == PrimitiveType::Vertex) {
        return 0;
    } else if (m_simplex_type == PrimitiveType::Vertex) {
        return target.identity();
    } else {
        return subgroup::convert(m_simplex_type, target.m_simplex_type, valid_index);
    }
}


} // namespace wmtk::autogen
