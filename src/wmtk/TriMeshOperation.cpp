
#include "TriMesh.hpp"
#include "Tuple.hpp"
#include "SimplicialComplex.hpp"
namespace wmtk
{
namespace {
    constexpr PrimitiveType PV = PrimitiveType::Vertex;
    constexpr PrimitiveType PE = PrimitiveType::Edge;
    constexpr PrimitiveType PF = PrimitiveType::Face;
}
class TriMesh::TriMeshOperationState
{
    public:
    TriMeshOperationState(TriMesh &m, const Tuple &operating_tuple);
    void delete_simplices();
    
    void merge();

    std::array<Accessor<char>, 3> flag_accessors;
    Accessor<long> ff_accessor;
    Accessor<long> fe_accessor;
    Accessor<long> fv_accessor;
    Accessor<long> vf_accessor;
    Accessor<long> ef_accessor;


    //           C
    //         /  \ 
    //    F1  /    \  F2 
    //       /      \ 
    //      /        \ 
    //     A----------B
    //      \        /
    //       \      /
    //    F1' \    / F2'
    //         \  /
    //          C'

    struct SimplexSet
    {
        long V_C_id;
        long E_AB_id;
        long E_BC_id;
        long E_AC_id;
        long F_ABC_id;
        long F1_id;
        long F2_id;
    };

    // common simplicies
    Simplex V_A, V_B, E_AB;
    long V_A_id, V_B_id, E_AB_id;
    // other simplicies    
    std::vector<SimplexSet> SimplexSets;

    std::vector<std::vector<long>> simplices_to_delete; // size 3 for vertex, edge, face
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
    , m_mesh(m)
    , V_A(PV, operating_tuple)
    , V_B(PV, m.switch_tuple(operating_tuple, PV))
    , E_AB(PE, operating_tuple)
{
    V_A_id = m_mesh.id(V_A.tuple(), PV);
    V_B_id = m_mesh.id(V_B.tuple(), PV);
    E_AB_id = m_mesh.id(E_AB.tuple(), PE);
    simplices_to_delete.resize(3);

    auto sw = [this](const Tuple& tt, const PrimitiveType& ptype) { return this->m_mesh.switch_tuple(tt, ptype); };

    auto get_simplexset = [&](Tuple he) -> SimplexSet {
        SimplexSet ret;
        Simplex V_C(PV, sw(sw(he, PE), PV));
        Simplex E_AB(PE, he), E_BC(PE, sw(sw(he, PV), PE)), E_AC(PE, sw(he, PE));
        Simplex FA(PF, he);
        
        ret.V_C_id = m_mesh.id(V_C.tuple(), PV);
        ret.E_AB_id = m_mesh.id(E_AB.tuple(), PE);
        ret.E_BC_id = m_mesh.id(E_BC.tuple(), PE);
        ret.E_AC_id = m_mesh.id(E_AC.tuple(), PE);
        ret.F_ABC_id = m_mesh.id(FA.tuple(), PF);

        for (long j = 0; j < 3; j++)
        {
            if (fe_accessor.vector_attribute(ret.F_ABC_id)[j] == ret.E_AC_id)
            {
                ret.F1_id = ff_accessor.vector_attribute(ret.F_ABC_id)[j];
            }
            if (fe_accessor.vector_attribute(ret.F_ABC_id)[j] == ret.E_BC_id)
            {
                ret.F2_id = ff_accessor.vector_attribute(ret.F_ABC_id)[j];
            }
        }
        return ret;
    };
 
    SimplexSets.emplace_back(get_simplexset(operating_tuple));
    if (!m_mesh.is_boundary(operating_tuple))
    {
        SimplexSets.emplace_back(get_simplexset(sw(operating_tuple, PF)));
    }
};

void TriMesh::TriMeshOperationState::delete_simplices()
{
    for (int d = 0; d < 3; d++)
    {
        for (long& simplex_id : simplices_to_delete[d]) {
            flag_accessors[d].scalar_attribute(simplex_id) = 0;
        }
    }
}


void TriMesh::TriMeshOperationState::merge() 
{
    simplices_to_delete[0].push_back(V_B_id);
    simplices_to_delete[1].push_back(E_AB_id);
    for (int set_ind = 0; set_ind < SimplexSets.size(); set_ind++)
    {
        if (SimplexSets[set_ind].F1_id < 0 && SimplexSets[set_ind].F2_id < 0)
        {
            return; // TODO: throw exception, should be detected by link condition
        }

        // change VF for V_A,V_C
        vf_accessor.scalar_attribute(V_A_id) = (SimplexSets[set_ind].F1_id < 0) ? SimplexSets[set_ind].F2_id : SimplexSets[set_ind].F1_id;
        vf_accessor.scalar_attribute(SimplexSets[set_ind].V_C_id) = vf_accessor.scalar_attribute(V_A_id);
        // change EF for E_AC
        ef_accessor.scalar_attribute(SimplexSets[set_ind].E_AC_id) = vf_accessor.scalar_attribute(V_A_id);
        // change FF and FE for  F1,F2
        auto change_ff_fe = [set_ind, this](long fid_one, long fid_other) {
            if (fid_one >= 0)
            {
                long index = 0;
                while (index < 2 && this->ff_accessor.vector_attribute(fid_one)[index] != this->SimplexSets[set_ind].F_ABC_id)
                {
                    index++;
                }
                assert(this->ff_accessor.vector_attribute(fid_one)[index] == this->SimplexSets[set_ind].F_ABC_id); // assert find F_ABC in FF(f_one)
                this->ff_accessor.vector_attribute(fid_one)[index] = fid_other;
                this->fe_accessor.vector_attribute(fid_one)[index] = this->SimplexSets[set_ind].E_AC_id;
            }
        };
        change_ff_fe(SimplexSets[set_ind].F1_id, SimplexSets[set_ind].F2_id);
        change_ff_fe(SimplexSets[set_ind].F2_id, SimplexSets[set_ind].F1_id);

        simplices_to_delete[1].push_back(SimplexSets[set_ind].E_BC_id);
        simplices_to_delete[2].push_back(SimplexSets[set_ind].F_ABC_id);
    }
};

void TriMesh::collapse_edge(const Tuple& t)
{
    // TODO: check link_cond before collapse
    TriMeshOperationState state(*this, t);

    // get faces in open_star(B)
    auto star_B_f = SimplicialComplex::open_star(state.V_B, *this).get_simplices(PF);
    std::vector<long> faces_in_open_star_B_id;
    for (const Simplex& simplex : star_B_f) 
    {
        faces_in_open_star_B_id.push_back(id(simplex.tuple(), PF));
    }

    // merge
    state.merge();

    // change FV for open_star_faces(V_B)
    for (long &f : faces_in_open_star_B_id)
    {
        for (long index = 0; index < 3; index++)
        {
            if (state.fv_accessor.vector_attribute(f)[index] == state.V_B_id)
            {
                state.fv_accessor.vector_attribute(f)[index] = state.V_A_id;
                break;
            }
        }
    }

    // delete simplices
    state.delete_simplices();
}

} // namespace wmtk