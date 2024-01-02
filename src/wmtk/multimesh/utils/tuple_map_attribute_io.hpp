#include <wmtk/Accessor.hpp>
#include <wmtk/Types.hpp>

namespace wmtk::multimesh::utils {

Vector<int64_t, 5> tuple_to_vector5(const Tuple& t);
Tuple vector5_to_tuple(const Vector5l& v);

// utility functions to simplify how we encode 2-Tuple attributes as 10-int64_t attribute
void write_tuple_map_attribute(
    Accessor<int64_t>& map_accessor,
    const Tuple& source_tuple,
    const Tuple& target_tuple);

std::tuple<Tuple, Tuple> read_tuple_map_attribute(
    const ConstAccessor<int64_t>& accessor,
    const Tuple& source_tuple);


void symmetric_write_tuple_map_attributes(
    Accessor<int64_t>& a_to_b,
    Accessor<int64_t>& b_to_a,
    const Tuple& a_tuple,
    const Tuple& b_tuple);
void write_tuple_map_attribute_slow(
    Mesh& source_mesh,
    TypedAttributeHandle<int64_t> map_handle,
    const Tuple& source_tuple,
    const Tuple& target_tuple);


std::tuple<Tuple, Tuple> read_tuple_map_attribute_slow(
    const Mesh& source_mesh,
    TypedAttributeHandle<int64_t> map_handle,
    const Tuple& source_tuple);
} // namespace wmtk::multimesh::utils
