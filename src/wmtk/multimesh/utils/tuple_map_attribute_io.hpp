#include <wmtk/Accessor.hpp>

namespace wmtk::multimesh::utils {


// utility functions to simplify how we encode 2-Tuple attributes as 10-long attribute
void write_tuple_map_attribute(
    Accessor<long>& map_accessor,
    const Tuple& source_tuple,
    const Tuple& target_tuple);

std::tuple<Tuple, Tuple> read_tuple_map_attribute(
    const ConstAccessor<long>& accessor,
    const Tuple& source_tuple);

void write_tuple_map_attribute_slow(
    Mesh& source_mesh,
    MeshAttributeHandle<long> map_handle,
    const Tuple& source_tuple,
    const Tuple& target_tuple);


std::tuple<Tuple, Tuple> read_tuple_map_attribute_slow(
    const Mesh& source_mesh,
    MeshAttributeHandle<long> map_handle,
    const Tuple& source_tuple);
} // namespace wmtk::multimesh::utils
