
#include "SimplicialComplex.hpp"
#include "TriMesh.hpp"
#include "Tuple.hpp"
namespace wmtk {
namespace {
constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;
} // namespace
class TriMesh::TriMeshOperationState
{
public:
    TriMeshOperationState(TriMesh& m, const Tuple& operating_tuple);
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
    // the neighbors are stored in the order of A, B, C, D if they exist
    // vid, ear fid (-1 if it doesn't exit), ear eid
    struct EarGlobalIDs
    {
        long fid = -1; // global fid of the ear, -1 if it doesn't exist
        long eid = -1; // global eid of the ear, -1 if it doesn't exist
    };
    struct PerFaceData
    {
        long V_C_id; // opposing vid
        long deleted_fid = -1; // the face that will be deleted
        std::array<EarGlobalIDs, 2> ears; // ear
    };


    // common simplicies
    std::array<long, 2> end_point_vids; // V_A_id, V_B_id;
    long E_AB_id;
    // simplices required per-face (other than those above)
    std::vector<PerFaceData> FaceDatas;

    std::vector<std::vector<long>> simplices_to_delete; // size 3 for vertex, edge, face
    TriMesh& m_mesh;
};

auto TriMesh::TriMeshOperationState::get_per_face_data(const Tuple& t) -> PerFaceData
{
    PerFaceData face_data;
    face_data.deleted_fid = m_mesh.id(t, PF);
    Tuple t1_edge = m_mesh.switch_tuple(t, PE);
    face_data.V_C_id = m_mesh.id(m_mesh.switch_tuple(t1_edge, PV), PV);
    Tuple t2_edge = m_mesh.switch_tuple(t, PV);
    t2_edge = m_mesh.switch_tuple(t2_edge, PE);

    per_face_data.ears[0] =
        EarGlobalIDs{.fid = ff_accessor.vector_attribute(t1_edge)(t1_edge.m_local_eid),
                     .eid = m_mesh.id(t1_edge, PE)};

    per_face_data.ears[1] =
        EarGlobalIDs{.fid = ff_accessor.vector_attribute(t2_edge)(t2_edge.m_local_eid),
                     .eid = m_mesh.id(t2_edge, PE)};

    return face_data;
}

// constructor
TriMesh::TriMeshOperationState::TriMeshOperationState(TriMesh& m, const Tuple& operating_tuple)
    : flag_accessors({{m.get_flag_accessor(PrimitiveType::Vertex),
                       m.get_flag_accessor(PrimitiveType::Edge),
                       m.get_flag_accessor(PrimitiveType::Face)}})
    , ff_accessor(m.create_accessor<long>(m.m_ff_handle))
    , fe_accessor(m.create_accessor<long>(m.m_fe_handle))
    , fv_accessor(m.create_accessor<long>(m.m_fv_handle))
    , vf_accessor(m.create_accessor<long>(m.m_ef_handle))
    , ef_accessor(m.create_accessor<long>(m.m_vf_handle))
    , m_mesh(m)

{
    end_point_vids[0] = m_mesh.id(operating_tuple, PV);
    end_point_vids[1] = m_mesh.id(m_mesh.switch_tuple(operating_tuple, PV), PV);

    E_AB_id = m_mesh.id(operating_tuple, PE);
    simplices_to_delete.resize(3);

    FaceDatas.emplace_back(get_per_face_data(operating_tuple));
    if (!m_mesh.is_boundary(operating_tuple)) {
        FaceDatas.emplace_back(get_per_face_data(m_mesh.switch_tuple(operating_tuple, PF)));
    }
};

void TriMesh::TriMeshOperationState::delete_simplices()
{
    for (int d = 0; d < 3; d++) {
        for (long& simplex_id : simplices_to_delete[d]) {
            flag_accessors[d].scalar_attribute(simplex_id) = 0;
        }
    }
}

/**
 * @brief handling the topology glueing of ear to non-ear face, transfering data from ear-oldface to
 * ear-newface
 *
 * @param ear_fid the ear that will be glued
 * @param new_face_fid
 * @param old_fid the data where the data is transfered from
 * @param eid the edge between two glued faces
 */
void TriMesh::TriMeshOperationState::glue_ear_to_face(
    const long ear_fid,
    const long new_face_fid,
    const long old_fid,
    const long eid) const
{
    if (ear_fid < 0) return;

    auto& ear_ff = ff_accessor.vector_attribute(ear_fid);
    auto& ear_fe = fe_accessor.vector_attribute(ear_fid);
    for (int i = 0; i < 3; i++) {
        if (ear_ff(i) == old_fid) {
            ear_ff(i) = new_face_fid;
            ear_fe(i) = eid;
            break;
        }
    }
}

void TriMesh::TriMeshOperationState::merge()
{
    simplices_to_delete[0].push_back(V_B_id);
    simplices_to_delete[1].push_back(E_AB_id);
    for (int set_ind = 0; set_ind < SimplexSets.size(); set_ind++) {
        if (SimplexSets[set_ind].F1_id < 0 && SimplexSets[set_ind].F2_id < 0) {
            return; // TODO: throw exception, should be detected by link condition
        }

        // change VF for V_A,V_C
        {
            long& vf_a = vf_accessor.scalar_attribute(V_A_id);
            const long f1 = SimplexSets[set_ind].F1_id;
            const long f2 = SimplexSets[set_ind].F2_id;
            vf_a = (f1 < 0) ? f2 : f1;
            vf_accessor.scalar_attribute(SimplexSets[set_ind].V_C_id) = vf_a;
        }
        // change EF for E_AC
        ef_accessor.scalar_attribute(SimplexSets[set_ind].E_AC_id) =
            vf_accessor.scalar_attribute(V_A_id);
        // change FF and FE for  F1,F2

        change_ff_fe(SimplexSets[set_ind], false);
        change_ff_fe(SimplexSets[set_ind], true);

        simplices_to_delete[1].push_back(SimplexSets[set_ind].E_BC_id);
        simplices_to_delete[2].push_back(SimplexSets[set_ind].F_ABC_id);
    }
};

void TriMeshOperationState::change_ff_fe(const SimplexSet& set, bool first)
{
    long my_fid = set.F1_id;
    long other_fid = set.F2_id;
    if (!first) {
        std::swap(my_fid, other_fid);
    }
    if (my_fid >= 0) {
        long index = 0;
        while (index < 2 && this->ff_accessor.vector_attribute(my_fid)[index] != set.F_ABC_id) {
            index++;
        }
        assert(
            this->ff_accessor.vector_attribute(my_fid)[index] ==
            set.F_ABC_id); // assert find F_ABC in FF(f_one)
        this->ff_accessor.vector_attribute(my_fid)[index] = other_fid;
        this->fe_accessor.vector_attribute(my_fid)[index] = set.E_AC_id;
    }
}

void TriMesh::collapse_edge(const Tuple& t)
{
    // TODO: check link_cond before collapse
    TriMeshOperationState state(*this, t);

    // get faces in open_star(B)
    auto star_B_f = SimplicialComplex::open_star(state.V_B, *this).get_simplices(PF);
    std::vector<long> faces_in_open_star_B_id;
    for (const Simplex& simplex : star_B_f) {
        faces_in_open_star_B_id.push_back(id(simplex.tuple(), PF));
    }

    // merge
    state.merge();

    // change FV for open_star_faces(V_B)
    for (long& f : faces_in_open_star_B_id) {
        for (long index = 0; index < 3; index++) {
            if (state.fv_accessor.vector_attribute(f)[index] == state.V_B_id)
                state.fv_accessor.vector_attribute(f)[index] = state.V_A_id;
            break;
        }
    }
}

// delete simplices
state.delete_simplices();
} // namespace wmtk


// void TriMesh::TriMeshOperationState::delete_simplices(const std::vector<wmtk::Simplex>&
// simplices)
// {
//     for (const auto& simplex_d : simplices) {
//         state.flag_accessors[simplex_d.dimension()].scalar_attribute(simplex_d.tuple()) = 0;
//     }
// }

void TriMesh::TriMeshOperationState::glue_new_triangle_topology(
    const long new_vid,
    const std::vector<long>& replacement_eids,
    const std::array<GlobalTuple, 2>& ears) const
{
    // create new faces
    std::vector<long> new_fids = m_mesh.request_simplex_indices(PrimitiveType::Face, 2);

    // create new edges
    std::vector<long> spine_edge = m_mesh.request_simplex_indices(PrimitiveType::Edge, 1);
    assert(spine_edge[0] > -1);
    long spine_eid = spine_edge[0];

    for (int i = 0; i < 2; ++i) {
        GlobalTuple& ear = ears[i];
        const GlobalTuple& other_ear = ears[(i + 1) % 2];
        const long my_fid = new_fids[i];
        const long other_fid = new_fids[(i + 1) % 2];
        // dummy transfer for new face attributes
        // this copies the deleted FF, FE, FV for two new faces. It handles the case of new to ear
        long deleted_fid = ear.deleted_fid;
        ff_accessor.vector_attribute(my_fid) = ff_accessor.vector_attribute(deleted_fid);
        fe_accessor.vector_attribute(my_fid) = fe_accessor.vector_attribute(deleted_fid);
        fv_accessor.vector_attribute(my_fid) = fv_accessor.vector_attribute(deleted_fid);

        auto ear_fe = fe_accessor.vector_attribute(ear.ear_fid);
        auto ear_ff = ff_accessor.vector_attribute(ear.ear_fid);
        auto my_fe = fe_accessor.vector_attribute(my_fid);
        auto my_ff = ff_accessor.vector_attribute(my_fid);
        auto my_fv = fv_accessor.vector_attribute(my_fid);

        for (int j = 0; j < 3; j++) {
            if (ear_fe(j) == ear.ear_eid) {
                // FF of the ear to new
                ear_ff(j) = my_fid;
            }

            if (my_fe(j) == other_ear.ear_eid) {
                // FF of the new to new
                my_ff(j) = other_fid;
                // FE of the new to new
                my_fe(j) = spine_eid;
            }

            if (my_fv(j) == other_ear.vid) {
                // FV of the new to new
                my_fv(j) = new_vid;
            }
        }
        // VF of the ear
        vf_accessor.scalar_attribute(ear.vid) = my_fid;
        // VF of the new
        vf_accessor.scalar_attribute(new_vid) = my_fid;
        // EF of the ear
        ef_accessor.scalar_attribute(ear.ear_eid) = my_fid;
        // EF of the spine
        ef_accessor.scalar_attribute(spine_eid) = my_fid;
        // EF of the new
        ef_accessor.scalar_attribute(replacement_eids[i]) = my_fid;
        // save the new fid to global cache
        ear.new_fid = my_fid;
        // set the new edge to be boundary for now
        // TODO: make sure that we cache this outdated neighbor insted of reading from it.
        // This assumes that the old fid is not re-used
        ff_accessor.vector_attribute(my_fid)(operating_tuple.m_local_eid) =
            ff_accessor.vector_attribute(ear.deleted_fid)(operating_tuple.m_local_eid);
    }
}

void TriMesh::split_edge(const Tuple& t)
{
    // record the deleted simplices topology attributes
    TriMesh::TriMeshOperationState state(*this, t);
    state.split_edge();
}
void TriMesh::TriMeshOperationState::split_edge()
{
    // delete star(edge)
    SimplicialComplex edge_open_star =
        wmtk::SimplicialComplex::open_star(Simplex(PrimitiveType::Edge, operating_tuple), *this);
    auto deleted_simplices = edge_open_star.get_simplex_vector();
    delete_simplices(deleted_simplices);

    // create new vertex
    std::vector<long> new_vids = request_simplex_indices(PrimitiveType::Vertex, 1);
    assert(new_vids.size() == 1);
    const long new_vid = new_vids[0];

    // create new edges
    std::vector<long> replacement_eids = request_simplex_indices(PrimitiveType::Edge, 2);
    assert(replacement_eids.size() == 2);


    for (int i = 0; i < ears_global_cache.size(); ++i) {
        const std::array<GlobalTuple, 2>& ears = ears_global_cache[i];
        // glue the topology
        glue_new_triangle_topology(new_vid, replacement_eids, ears);
    }
    assert(ears_global_cache.size() <= 2);
    // update FF on two sides of ab
    if (ears_global_cache.size() > 1) {
        const std::array<GlobalTuple, 2>& earsi = ears_global_cache[0];
        const std::array<GlobalTuple, 2>& earsj = ears_global_cache[1];
        ff_accessor.vector_attribute(earsi[0].new_fid)(t.m_local_eid) = earsi[0].new_fid;
        ff_accessor.vector_attribute(earsj[0].new_fid)(t.m_local_eid) = earsj[0].new_fid;
        ff_accessor.vector_attribute(earsi[1].new_fid)(t.m_local_eid) = earsi[1].new_fid;
        ff_accessor.vector_attribute(earsj[1].new_fid)(t.m_local_eid) = earsj[1].new_fid;
    }
}


} // namespace wmtk
