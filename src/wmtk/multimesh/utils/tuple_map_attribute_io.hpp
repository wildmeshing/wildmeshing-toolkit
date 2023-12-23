#include <wmtk/Accessor.hpp>
#include <wmtk/Types.hpp>

namespace wmtk::multimesh::utils {

Vector<long, 5> tuple_to_vector5(const Tuple& t);
Tuple vector5_to_tuple(const Vector5l& v);

// utility functions to simplify how we encode 2-Tuple attributes as 10-long attribute
void write_tuple_map_attribute(
    Accessor<long>& map_accessor,
    const Tuple& source_tuple,
    const Tuple& target_tuple);

std::tuple<Tuple, Tuple> read_tuple_map_attribute(
    const ConstAccessor<long>& accessor,
    const Tuple& source_tuple);


void symmetric_write_tuple_map_attributes(
    Accessor<long>& a_to_b,
    Accessor<long>& b_to_a,
    const Tuple& a_tuple,
    const Tuple& b_tuple);
void write_tuple_map_attribute_slow(
    Mesh& source_mesh,
    TypedAttributeHandle<long> map_handle,
    const Tuple& source_tuple,
    const Tuple& target_tuple);


std::tuple<Tuple, Tuple> read_tuple_map_attribute_slow(
    const Mesh& source_mesh,
    TypedAttributeHandle<long> map_handle,
    const Tuple& source_tuple);
} // namespace wmtk::multimesh::utils
