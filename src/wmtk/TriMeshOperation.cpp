
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

    Simplex V_A(PV, t), V_B(PV, sw(t, PV));
    long V_A_id = id(V_A.tuple(), PV);
    long V_B_id = id(V_B.tuple(), PV);
    // get faces in open_star(B)
    auto star_B_f = SimplicialComplex::open_star(V_B, *this).get_simplices(PF);
    std::vector<Simplex> faces_in_open_star_B = std::vector<Simplex>(star_B_f.begin(), star_B_f.end());
    std::vector<long> faces_in_open_star_B_id;
    for (const Simplex& simplex : faces_in_open_star_B) 
    {
        faces_in_open_star_B_id.push_back(id(simplex.tuple(), PF));
    }

    std::vector<Simplex> simplices_to_delete;
    auto merge_he = [&](const Tuple &input_he, int ear_ind) {
        Simplex V_C(PV, sw(sw(input_he, PE), PV));
        Simplex E_AB(PE, input_he), E_BC(PE, sw(sw(input_he, PV), PE)), E_AC(PE, sw(input_he, PE));
        Simplex FA(PF, input_he);
        
        long V_C_id = id(V_C.tuple(), PV);
        long E_AB_id = id(E_AB.tuple(), PE);
        long E_BC_id = id(E_BC.tuple(), PE);
        long E_AC_id = id(E_AC.tuple(), PE);
        long FA_id = id(FA.tuple(), PF);
        long f1_id = state.edge_neighbors[ear_ind][0].has_value() ? id(state.edge_neighbors[ear_ind][0].value(), PF) : -1;
        long f2_id = state.edge_neighbors[ear_ind][1].has_value() ? id(state.edge_neighbors[ear_ind][1].value(), PF) : -1;
        if (f1_id == -1 && f2_id == -1)
        {
            return; // TODO: throw exception
        }

        // change VF for V_A,V_C
        state.vf_accessor.scalar_attribute(V_A_id) = (f1_id == -1) ? f2_id : f1_id;
        state.vf_accessor.scalar_attribute(V_C_id) = state.vf_accessor.scalar_attribute(V_A_id);
        // change EF for E_AC
        state.ef_accessor.scalar_attribute(E_AC_id) = state.vf_accessor.scalar_attribute(V_A_id);
        
        // change FF and FE for  F1,F2
        auto change_ff_fe = [this, &state, &E_AC_id, &FA_id](long fid_one, long fid_another) {
            if (fid_one != -1)
            {
                long index = 0;
                while (index < 2 && state.ff_accessor.vector_attribute(fid_one)[index] != FA_id)
                {
                    index++;
                }
                assert(state.ff_accessor.vector_attribute(fid_one)[index] == FA_id); // assert find FA in FF(f1)
                state.ff_accessor.vector_attribute(fid_one)[index] = fid_another;
                state.fe_accessor.vector_attribute(fid_one)[index] = E_AC_id;
            }
        };
        change_ff_fe(f1_id, f2_id);
        change_ff_fe(f2_id, f1_id);

        simplices_to_delete.push_back(V_B);
        simplices_to_delete.push_back(E_BC);
        simplices_to_delete.push_back(E_AB);
        simplices_to_delete.push_back(FA);
    };
    
    // merge
    merge_he(t, 0);
    if (!is_t_boudanry)
    {
        merge_he(sw(t, PF), 1);
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