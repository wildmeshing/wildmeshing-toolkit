#include <wmtk/operations/TriMeshVertexSmoothOperation.h>

namespace wmtk {
bool TriMeshVertexSmoothOperation::execute(TriMesh& m, const Tuple& t)
{
    set_return_tuple(t);
    // always succeed and return the Tuple for the (vertex) that we pointed at
    return true;
}
auto TriMeshVertexSmoothOperation::modified_triangles(const TriMesh& m) const -> std::vector<Tuple>
{
    const auto& new_tup_opt = get_return_tuple_opt();
    assert(new_tup_opt);
    return m.get_one_ring_tris_for_vertex(new_tup_opt.value());
}
bool TriMeshVertexSmoothOperation::before(TriMesh& m, const Tuple& t)
{
    return true;
}
bool TriMeshVertexSmoothOperation::after(TriMesh& m)
{
    return true;
}
std::string TriMeshVertexSmoothOperation::name() const
{
    return "vertex_smooth";
}

// bool TriMeshVertexSmoothOperation::invariants(TriMesh& m, ExecuteReturnData& ret_data)
// {
//     // todo: mtao: figure out how to incorporate this properly
//     //  our execute should have tuple set to the input tuple (vertex)
//     // return TriMesh::invariants(m.get_one_ring_tris_for_vertex(ret_data.tuple));
//     return m.invariants();
// }
} // namespace wmtk
