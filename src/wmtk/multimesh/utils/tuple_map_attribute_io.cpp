#include "tuple_map_attribute_io.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/utils/TupleInspector.hpp>

#include <wmtk/utils/Logger.hpp>

namespace wmtk::multimesh::utils {

Vector<int64_t, 5> tuple_to_vector5(const Tuple& t)
{
    Vector<int64_t, 5> v;
    v(0) = wmtk::utils::TupleInspector::local_vid(t);
    v(1) = wmtk::utils::TupleInspector::local_eid(t);
    v(2) = wmtk::utils::TupleInspector::local_fid(t);
    v(3) = wmtk::utils::TupleInspector::global_cid(t);
    v(4) = wmtk::utils::TupleInspector::hash(t);
    return v;
}

Tuple vector5_to_tuple(const Vector5l& v)
{
    return Tuple(v(0), v(1), v(2), v(3), v(4));
}


void symmetric_write_tuple_map_attributes(
    Accessor<int64_t>& a_to_b,
    Accessor<int64_t>& b_to_a,
    const Tuple& a_tuple,
    const Tuple& b_tuple)
{
    logger().debug(
        "[{} -> {}] Symmetric map write parent {}  child {}",
        fmt::join(a_to_b.mesh().absolute_multi_mesh_id(), ","),
        fmt::join(b_to_a.mesh().absolute_multi_mesh_id(), ","),
        wmtk::utils::TupleInspector::as_string(a_tuple),
        wmtk::utils::TupleInspector::as_string(b_tuple));
    write_tuple_map_attribute(a_to_b, a_tuple, b_tuple);
    write_tuple_map_attribute(b_to_a, b_tuple, a_tuple);
}
void write_tuple_map_attribute(
    Accessor<int64_t>& map_accessor,
    const Tuple& source_tuple,
    const Tuple& target_tuple)
{
    auto map = map_accessor.vector_attribute(source_tuple);

    assert(map.size() == 10);
    map.head<5>() = tuple_to_vector5(source_tuple);
    map.tail<5>() = tuple_to_vector5(target_tuple);
}

void write_tuple_map_attribute_slow(
    Mesh& source_mesh,
    TypedAttributeHandle<int64_t> map_handle,
    const Tuple& source_tuple,
    const Tuple& target_tuple)
{
    auto map_accessor = source_mesh.create_accessor(map_handle);
    write_tuple_map_attribute(map_accessor, source_tuple, target_tuple);
}

std::tuple<Tuple, Tuple> read_tuple_map_attribute(
    const ConstAccessor<int64_t>& map_accessor,
    const Tuple& source_tuple)
{
    auto map = map_accessor.const_vector_attribute(source_tuple);

    return std::make_tuple(vector5_to_tuple(map.head<5>()), vector5_to_tuple(map.tail<5>()));
}


std::tuple<Tuple, Tuple> read_tuple_map_attribute_slow(
    const Mesh& source_mesh,
    TypedAttributeHandle<int64_t> map_handle,
    const Tuple& source_tuple)
{
    auto acc = source_mesh.create_const_accessor(map_handle);
    return read_tuple_map_attribute(acc, source_tuple);
}
} // namespace wmtk::multimesh::utils
