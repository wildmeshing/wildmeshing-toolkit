
#include "TriMesh.hpp"
#include "Tuple.hpp"
#include "SimplicialComplex.hpp"
namespace wmtk
{
class TriMesh::TriMeshOperationState
{
    public:
    TriMeshOperationState(TriMesh &m, const Tuple &operating_tuple);
    void delete_simplices(const std::vector<Simplex>& simplicess);

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
    const std::vector<wmtk::Simplex>& simplices)
{
    for (const Simplex& simplex : simplices) {
        flag_accessors[simplex.dimension()].scalar_attribute(simplex.tuple()) = 0;
    }
}

void TriMesh::collapse_edge(const Tuple& t)
{
    // TODO: check link_cond before collapse

    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Face;
    auto sw = [this](const Tuple& tt, const PrimitiveType& ptype) { return this->switch_tuple(tt, ptype); };
    
    bool is_t_boudanry = is_boundary(t);
    TriMeshOperationState state(*this, t);
    
    Simplex V_A(PV, t), V_B(PV, sw(t, PV)), V_C(PV, sw(sw(t, PE), PV));
    Simplex E_AB(PE, t), E_BC(PE, sw(sw(t, PV), PE)), E_AC(PE, sw(t, PE));
    Simplex FA(PF, t);
    long V_A_id = id(V_A.tuple(), PV);
    long V_B_id = id(V_B.tuple(), PV);
    long V_C_id = id(V_C.tuple(), PV);
    long E_AB_id = id(E_AB.tuple(), PE);
    long E_BC_id = id(E_BC.tuple(), PE);
    long E_AC_id = id(E_AC.tuple(), PE);
    long FA_id = id(FA.tuple(), PF);

    // get faces in open_star(B)
    auto star_B_f = SimplicialComplex::open_star(V_B, *this).get_simplices(PF);
    std::vector<Simplex> faces_in_open_star_B = std::vector<Simplex>(star_B_f.begin(), star_B_f.end());
    std::vector<long> faces_in_open_star_B_id;
    for (const Simplex& simplex : faces_in_open_star_B) 
    {
        faces_in_open_star_B_id.push_back(id(simplex.tuple(), PF));
    }

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

    // get ears
    long f1_id = state.edge_neighbors[0][0].has_value() ? id(state.edge_neighbors[0][0].value(), PF) : -1;
    long f2_id = state.edge_neighbors[0][1].has_value() ? id(state.edge_neighbors[0][1].value(), PF) : -1;
    if (f1_id == -1 && f2_id == -1)
    {
        return; // TODO: throw exception
    }
    long f3_id, f4_id;
    if (!is_t_boudanry)
    {
        f3_id = state.edge_neighbors[1][0].has_value() ? id(state.edge_neighbors[1][0].value(), PF) : -1;
        f4_id = state.edge_neighbors[1][1].has_value() ? id(state.edge_neighbors[1][1].value(), PF) : -1;
        if (f3_id == -1 && f4_id == -1)
        {
            return; // TODO: throw exception
        } 
    }
    
    // change VF for V_A,V_C,(V_D)
    state.vf_accessor.scalar_attribute(V_A_id) = (f1_id == -1) ? f2_id : f1_id;
    state.vf_accessor.scalar_attribute(V_C_id) = state.vf_accessor.scalar_attribute(V_A_id);
    if (!is_t_boudanry)
    {
        long V_D_id = simplices_other_side_id[3];
        state.vf_accessor.scalar_attribute(V_D_id) = (f3_id == -1) ? f4_id : f3_id;
    }
    // change EF for E_AC,(E_AD)
    state.ef_accessor.scalar_attribute(E_AC_id) = state.vf_accessor.scalar_attribute(V_A_id);
    if (!is_t_boudanry)
    {
        long E_AD_id = simplices_other_side_id[1];
        long V_D_id = simplices_other_side_id[3];
        state.ef_accessor.scalar_attribute(E_AD_id) = state.vf_accessor.scalar_attribute(V_D_id);
    }
    
    // change FF and FE for  F1,F2,(F3,F4)
    if (f1_id != -1)
    {
        long index = 0;
        while (index < 2 && state.ff_accessor.vector_attribute(f1_id)[index] != FA_id)
        {
            index++;
        }
        assert(state.ff_accessor.vector_attribute(f1_id)[index] == FA_id); // assert find FA in FF(f1)
        state.ff_accessor.vector_attribute(f1_id)[index] = f2_id;
        state.fe_accessor.vector_attribute(f1_id)[index] = E_AC_id;
    }
    if (f2_id != -1)
    {
        long index = 0;
        while (index < 2 && state.ff_accessor.vector_attribute(f2_id)[index] != FA_id)
        {
            index++;
        }
        assert(state.ff_accessor.vector_attribute(f2_id)[index] == FA_id); // assert find FA in FF(f2)
        state.ff_accessor.vector_attribute(f2_id)[index] = f1_id;
        state.fe_accessor.vector_attribute(f2_id)[index] = E_AC_id;
    }
    if (!is_t_boudanry)
    {
        long FB_id = simplices_other_side_id[0];
        long E_AD_id = simplices_other_side_id[1];
        if (f3_id != -1)
        {
            long index = 0;
            while (index < 2 && state.ff_accessor.vector_attribute(f3_id)[index] != FB_id)
            {
                index++;
            }
            assert(state.ff_accessor.vector_attribute(f3_id)[index] == FB_id); // assert find FB in FF(f3)
            state.ff_accessor.vector_attribute(f3_id)[index] = f4_id;
            state.fe_accessor.vector_attribute(f3_id)[index] = E_AD_id;
        }
        if (f4_id != -1)
        {
            long index = 0;
            while (index < 2 && state.ff_accessor.vector_attribute(f4_id)[index] != FB_id)
            {
                index++;
            }
            assert(state.ff_accessor.vector_attribute(f4_id)[index] == FB_id); // assert find FB in FF(f4)
            state.ff_accessor.vector_attribute(f4_id)[index] = f3_id;
            state.fe_accessor.vector_attribute(f4_id)[index] = E_AD_id;
        }
    }

    // change FV for open_star_faces(V_B)
    for (long &f : faces_in_open_star_B_id)
    {
        for (long index = 0; index < 3; index++)
        {
            if (state.fv_accessor.vector_attribute(f)[index] == V_B_id)
            {
                state.fv_accessor.vector_attribute(f)[index] = V_A_id;
                break;
            }
        }
    }

    // delete simplices
    state.delete_simplices(simplices_to_delete);
}

} // namespace wmtk