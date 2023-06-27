
#include "TriMesh.hpp"
#include "Tuple.hpp"
#include "SimplicialComplex.hpp"
namespace wmtk
{
class TriMesh::TriMeshOperationState
{
    TriMeshOperationState(TriMesh &m, Tuple operating_tuple);
    void delete_simplices(
    const std::vector<std::vector<wmtk::Simplex>>& simplices,
    TriMesh::TriMeshOperationState& state);

    std::array<Accessor<char>, 3> flag_accessors;

    Accessor<long> ff_accessor;
    Accessor<long> fe_accessor;
    Accessor<long> fv_accessor;
    Accessor<long> vf_accessor;
    Accessor<long> ef_accessor;

    Tuple edge_tuple;

    //         /  \ 
    //      A /    \ B
    //       /      \ 
    //      /        \ 
    //      ----------
    //      \        /
    //       \      /
    //      C \    / D
    //         \  /

    // the neighbors are stored in the order of A, B, C, D if they exist
    std::vector<std::array<std::optional<Tuple>, 2>> edge_neighbors;
    TriMesh &m_mesh;
};

// constructor
TriMesh::TriMeshOperationState::TriMeshOperationState(TriMesh &m, Tuple operating_tuple)
    : flag_accessors({{m.get_flag_accessor(PrimitiveType::Vertex),
                       m.get_flag_accessor(PrimitiveType::Edge),
                       m.get_flag_accessor(PrimitiveType::Face)}})
    , ff_accessor(m.create_accessor<long>(m.m_ff_handle))
    , fe_accessor(m.create_accessor<long>(m.m_fe_handle))
    , fv_accessor(m.create_accessor<long>(m.m_fv_handle))
    , vf_accessor(m.create_accessor<long>(m.m_ef_handle))
    , ef_accessor(m.create_accessor<long>(m.m_vf_handle))
    , edge_tuple(operating_tuple)
    , m_mesh(m)
{
    auto get_ears = [&](Tuple edge_tuple) -> std::array<std::optional<Tuple>, 2> {
        std::array<std::optional<Tuple>, 2> ears;
        Tuple t1_edge = m_mesh.switch_tuple(edge_tuple, PrimitiveType::Edge);
        Tuple t2_edge = m_mesh.switch_tuple(edge_tuple, PrimitiveType::Vertex);
        t2_edge = m_mesh.switch_tuple(t2_edge, PrimitiveType::Edge);

        ears[0] = m_mesh.is_boundary(t1_edge)
                      ? std::nullopt
                      : std::optional<Tuple>(m_mesh.switch_tuple(t1_edge, PrimitiveType::Face));
        ears[1] = m_mesh.is_boundary(t2_edge)
                      ? std::nullopt
                      : std::optional<Tuple>(m_mesh.switch_tuple(t2_edge, PrimitiveType::Face));
        return ears;
    };
    edge_neighbors.emplace_back(get_ears(operating_tuple));

    if (!m_mesh.is_boundary(operating_tuple)) {
        Tuple other_face = m_mesh.switch_tuple(operating_tuple, PrimitiveType::Face);
        edge_neighbors.emplace_back(get_ears(other_face));
    }
};

void TriMesh::TriMeshOperationState::delete_simplices(
    const std::vector<std::vector<wmtk::Simplex>>& simplices,
    TriMesh::TriMeshOperationState& state)
{
    for (const auto& simplices_d : simplices) {
        for (const Simplex& simplex : simplices_d) {
            state.flag_accessors[simplex.dimension()].scalar_attribute(simplex.tuple()) = 0;
        }
    }
}

} // namespace wmtk