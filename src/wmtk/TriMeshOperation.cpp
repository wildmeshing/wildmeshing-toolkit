
#include "TriMesh.hpp"
#include "Tuple.hpp"
#include "SimplicialComplex.hpp"
namespace wmtk
{
class TriMesh::TriMeshOperationState
{
    public:
    TriMeshOperationState(TriMesh &m, const Tuple &operating_tuple);
    void delete_simplices( const std::vector<Simplex>& simplices, TriMesh::TriMeshOperationState& state);

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
TriMesh::TriMeshOperationState::TriMeshOperationState(TriMesh &m, const Tuple &operating_tuple)
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
    auto get_ears = [&](Tuple e_tuple) -> std::array<std::optional<Tuple>, 2> {
        std::array<std::optional<Tuple>, 2> ears;
        Tuple t1_edge = m_mesh.switch_tuple(e_tuple, PrimitiveType::Edge);
        Tuple t2_edge = m_mesh.switch_tuple(e_tuple, PrimitiveType::Vertex);
        t2_edge = m_mesh.switch_tuple(t2_edge, PrimitiveType::Edge);

        ears[0] = m_mesh.is_boundary(t1_edge)
                      ? std::nullopt
                      : std::optional<Tuple>(m_mesh.switch_tuple(t1_edge, PrimitiveType::Face));
        ears[1] = m_mesh.is_boundary(t2_edge)
                      ? std::nullopt
                      : std::optional<Tuple>(m_mesh.switch_tuple(t2_edge, PrimitiveType::Face));
        return ears;
    };
    edge_neighbors.emplace_back(get_ears(edge_tuple));

    if (!m_mesh.is_boundary(edge_tuple)) {
        Tuple other_face = m_mesh.switch_tuple(edge_tuple, PrimitiveType::Face);
        edge_neighbors.emplace_back(get_ears(other_face));
    }
};

void TriMesh::TriMeshOperationState::delete_simplices(
    const std::vector<wmtk::Simplex>& simplices,
    TriMesh::TriMeshOperationState& state)
{
    for (const Simplex& simplex : simplices) {
        state.flag_accessors[simplex.dimension()].scalar_attribute(simplex.tuple()) = 0;
    }
}

void TriMesh::collapse_edge(const Tuple& t)
{
    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Face;
    auto sw = [this](const Tuple& tt, const PrimitiveType& ptype) { return this->switch_tuple(tt, ptype); };
    
    bool is_t_boudanry = is_boundary(t);
    TriMeshOperationState state(*this, t);
    
    Simplex V_A(PV, t), V_B(PV, sw(t, PV));
    Simplex E_AB(PE, t), E_BC(PE, sw(sw(t, PV), PE)), E_AC(PE, sw(t, PE));
    Simplex FA(PF, t);
    long V_A_id = id(V_A.tuple(), PV);
    long V_B_id = id(V_B.tuple(), PV);
    long E_AB_id = id(E_AB.tuple(), PE);
    long E_BC_id = id(E_BC.tuple(), PE);
    long E_AC_id = id(E_AC.tuple(), PE);
    long FA_id = id(FA.tuple(), PF);

    std::vector<Simplex> simplices_other_side;
    std::vector<long> simplices_other_side_id;
    if (!is_t_boudanry)
    {
        // FB
        simplices_other_side.push_back(Simplex(PF, sw(t, PF)));
        simplices_other_side_id.push_back(id(simplices_other_side.back().tuple(), PF));
        // AD
        simplices_other_side.push_back(Simplex(PE, sw(sw(t, PF), PE)));
        simplices_other_side_id.push_back(id(simplices_other_side.back().tuple(), PE));
        // BD
        simplices_other_side.push_back(Simplex(PE, sw(sw(sw(t, PF), PV), PE)));
        simplices_other_side_id.push_back(id(simplices_other_side.back().tuple(), PE));
        // D
        simplices_other_side.push_back(Simplex(PV, sw(sw(sw(t, PF), PE), PV)));
        simplices_other_side_id.push_back(id(simplices_other_side.back().tuple(), PV));

    }

    std::vector<Simplex> simplices_to_delete;
    simplices_to_delete.push_back(V_B);
    simplices_to_delete.push_back(E_BC);
    simplices_to_delete.push_back(E_AB);
    simplices_to_delete.push_back(FA);
    if (!is_t_boudanry)
    {
        simplices_to_delete.push_back(simplices_other_side[0]); // FB
        simplices_to_delete.push_back(simplices_other_side[2]); // BD
    }
}

} // namespace wmtk