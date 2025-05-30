#include "tuple_map_attribute_io.hpp"
#include <wmtk/attribute/TupleAccessor.hpp>

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Types.hpp>

#include <wmtk/utils/Logger.hpp>

namespace wmtk::utils {


namespace {
using Vec84 = Vector<int8_t, 4>;
}
Vector<int64_t, 2> tuple_to_vector2(const Tuple& t)
{
    Vector<int64_t, 2> v = Vector<int64_t, 2>::ConstMapType(reinterpret_cast<const int64_t*>(&t));
    return v;
}

Tuple vector2_to_tuple(const Eigen::Ref<const Vector2l>& v)
{
    const Tuple* data = reinterpret_cast<const Tuple*>(v.data());
    return *data;
}
Vector<int64_t, 4> tuple_to_vector5(const Tuple& t)
{
    Vector<int64_t, 4> v;
    v(0) = t.local_vid();
    v(1) = t.local_eid();
    v(2) = t.local_fid();
    v(3) = t.global_cid();
    return v;
}

Tuple vector5_to_tuple(const Eigen::Ref<const Vector5l>& v)
{
    return Tuple(v(0), v(1), v(2), v(3));
}

Vector<int64_t, TUPLE_SIZE> tuple_to_vector(const Tuple& t)
{
#if defined WMTK_DISABLE_COMPRESSED_MULTIMESH_TUPLE
    return tuple_to_vector5(t);
#else
    return tuple_to_vector2(t);
#endif
}
Tuple vector_to_tuple(const Eigen::Ref<const TupleVector>& v)
{
#if defined WMTK_DISABLE_COMPRESSED_MULTIMESH_TUPLE
    return vector5_to_tuple(v);
#else
    return vector2_to_tuple(v);
#endif
}

std::tuple<Tuple, Tuple> vectors_to_tuples(const Eigen::Ref<const TwoTupleVector>& v)
{
    return std::make_tuple(
        vector_to_tuple(v.head<TUPLE_SIZE>()),
        vector_to_tuple(v.tail<TUPLE_SIZE>()));
}
TwoTupleVector tuples_to_vectors(const Tuple& a, const Tuple& b)
{
    TwoTupleVector t;
    t.head<TUPLE_SIZE>() = tuple_to_vector(a);
    t.tail<TUPLE_SIZE>() = tuple_to_vector(b);
    return t;
}


template <typename MeshType>
std::tuple<Tuple, Tuple> read_tuple_map_attribute(
    const wmtk::attribute::Accessor<int64_t, MeshType>& accessor,
    const Tuple& source_tuple)
{
#if !defined(NDEBUG)
    // check t hat the global ids are either -1 or positive
    auto v = accessor.const_vector_attribute(source_tuple);
    assert(v(0) >= -1);
    assert(v(2) >= -1);
#endif
    assert(accessor.dimension() == 4);

    const wmtk::attribute::TupleAccessor<MeshType> acc(accessor);
    assert(acc.dimension() == 2);
    auto map = acc.const_vector_attribute(source_tuple);
    return std::tie(map(0), map(1));
}


template std::tuple<Tuple, Tuple> read_tuple_map_attribute(
    const wmtk::attribute::Accessor<int64_t, wmtk::Mesh>& accessor,
    const Tuple& source_tuple);
template std::tuple<Tuple, Tuple> read_tuple_map_attribute(
    const wmtk::attribute::Accessor<int64_t, wmtk::EdgeMesh>& accessor,
    const Tuple& source_tuple);
template std::tuple<Tuple, Tuple> read_tuple_map_attribute(
    const wmtk::attribute::Accessor<int64_t, wmtk::TriMesh>& accessor,
    const Tuple& source_tuple);
template std::tuple<Tuple, Tuple> read_tuple_map_attribute(
    const wmtk::attribute::Accessor<int64_t, wmtk::TetMesh>& accessor,
    const Tuple& source_tuple);


template <typename MeshType>
void write_tuple_map_attribute(
    wmtk::attribute::Accessor<int64_t, MeshType>& map_accessor,
    const Tuple& source_tuple,
    const Tuple& target_tuple)
{
    assert(map_accessor.dimension() == 4);
    wmtk::attribute::TupleAccessor<MeshType> acc(map_accessor);
    assert(acc.dimension() == 2);
    auto map = acc.vector_attribute(source_tuple);
    map(0) = source_tuple;
    map(1) = target_tuple;
#if !defined(NDEBUG)
    // check t hat the global ids are either -1 or positive
    auto v = map_accessor.const_vector_attribute(source_tuple);
    assert(v(0) >= -1);
    assert(v(2) >= -1);
#endif
}

template void write_tuple_map_attribute(
    wmtk::attribute::Accessor<int64_t, wmtk::Mesh>& map_accessor,
    const Tuple& source_tuple,
    const Tuple& target_tuple);
template void write_tuple_map_attribute(
    wmtk::attribute::Accessor<int64_t, wmtk::EdgeMesh>& map_accessor,
    const Tuple& source_tuple,
    const Tuple& target_tuple);
template void write_tuple_map_attribute(
    wmtk::attribute::Accessor<int64_t, wmtk::TriMesh>& map_accessor,
    const Tuple& source_tuple,
    const Tuple& target_tuple);
template void write_tuple_map_attribute(
    wmtk::attribute::Accessor<int64_t, wmtk::TetMesh>& map_accessor,
    const Tuple& source_tuple,
    const Tuple& target_tuple);


void write_tuple_map_attribute_slow(
    Mesh& source_mesh,
    TypedAttributeHandle<int64_t> map_handle,
    const Tuple& source_tuple,
    const Tuple& target_tuple)
{
    auto map_accessor = source_mesh.create_accessor(map_handle);
    write_tuple_map_attribute(map_accessor, source_tuple, target_tuple);
}
std::tuple<Tuple, Tuple> read_tuple_map_attribute_slow(
    const Mesh& source_mesh,
    TypedAttributeHandle<int64_t> map_handle,
    const Tuple& source_tuple)
{
    auto acc = source_mesh.create_const_accessor(map_handle);
    return read_tuple_map_attribute(acc, source_tuple);
}
} // namespace wmtk::utils
