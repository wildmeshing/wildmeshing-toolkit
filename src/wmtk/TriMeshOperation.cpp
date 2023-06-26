
#include "TriMesh.hpp"
#include "Tuple.hpp"

class TriMesh::TriMeshOperationSate
{
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
    std::vector<std::array<std::optional<Tuple>, 2>> neighbors;
};

// constructor
TriMeshOperationSate::TriMeshOperationState(Tuple operating_tuple)
    : flag_accessors({{get_flag_accessor<char>(PrimitiveType::Vertex),
                       get_flag_accessor<char>(PrimitiveType::Edge),
                       get_flag_accessor<char>(PrimitiveType::Face)}})
    , ff_accessor(create_accessor<long>(m_ff_handle))
    , fe_accessor(create_accessor<long>(m_fe_handle))
    , fv_accessor(create_accessor<long>(m_fv_handle))
    , vf_accessor(create_accessor<long>(m_ef_handle))
    , ef_accessor(create_accessor<long>(m_vf_handle))
    , edge_tuple(operating_tuple)
{
    auto get_ears = [&](Tuple edge_tuple) -> std::array<std::optional<Tuple>, 2> {
        std::array<Tuple, 2> ears;
        Tuple t1_edge = switch_tuple(edge_tuple, PrimitiveType::Edge);
        Tuple t2_edge = switch_tuple(edge_tuple, PrimitiveType::Vertex);
        t2_edge = switch_tuple(t2_edge, PrimitiveType::Edge);

        ears[0] = is_boundary(t1_edge)
                      ? std::nullopt
                      : std::optional<Tuple>(switch_tuple(t1_edge, PrimitiveType::Face));
        ears[1] = is_boundary(t2_edge)
                      ? std::nullopt
                      : std::optional<Tuple>(switch_tuple(t2_edge, PrimitiveType::Face));
        return ears;
    };
    neighbors.emplace_back(get_ears(operating_tuple));

    if (!is_boundary(operating_edge)) {
        Tuple other_face = switch_tuple(operating_edge, PrimitiveType::Face);
        neighbors.emplace_back(get_ears(other_face));
    }
}

void TriMesh::TriMeshOpertaionState::delete_simplices(
    const std::vector<std::vector<Simplex>>& simplices,
    TriMesh::TriMeshOperationState& state)
{
    for (const auto& simplices_d : simplices) {
        for (const Simplex& simplex : simplices_d) {
            state.face_flag_accessor.scalar_attribute(simplex.tuple()) = 0;
        }
    }
}