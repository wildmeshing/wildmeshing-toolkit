#include "get_edge_tuple.hpp"
#include <wmtk/autogen/tri_mesh/get_tuple_from_simplex_local_id.hpp>
wmtk::Tuple get_edge_tuple(
    const wmtk::attribute::Accessor<int64_t, wmtk::TriMesh, 3>& FV,
    int64_t global_id,
    const int64_t a,
    const int64_t b)
{
    auto fv = FV.index_access().const_vector_attribute(global_id);
    for (int j = 0; j < 3; ++j) {
        if (j != a && j != b) {
            return wmtk::autogen::tri_mesh::get_tuple_from_simplex_local_edge_id(global_id, j);
        }
    }
    assert(false);
}
