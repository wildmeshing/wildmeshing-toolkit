
#include "TopoOffsetMesh.h"

#include "wmtk/utils/Rational.hpp"

#include <wmtk/utils/AMIPS.h>
#include <wmtk/envelope/KNN.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TetraQualityUtils.hpp>
#include <wmtk/utils/io.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/concurrent_vector.h>
#include <spdlog/fmt/ostr.h>
#include <spdlog/fmt/bundled/format.h>
#include <igl/predicates/predicates.h>
#include <igl/winding_number.h>
#include <igl/write_triangle_mesh.h>
#include <igl/remove_unreferenced.h>
#include <igl/Timer.h>
#include <igl/orientable_patches.h>
#include <wmtk/utils/EnableWarnings.hpp>
#include <wmtk/utils/GeoUtils.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
// clang-format on

#include <paraviewo/VTUWriter.hpp>

#include <limits>


namespace {
static int debug_print_counter = 0;
}

namespace wmtk::components::topological_offset {


VertexAttributes::VertexAttributes(const Vector3d& p)
    : m_posf(p)
{}


void TopoOffsetMesh::init_from_image(const MatrixXd& V, const MatrixXi& T, const MatrixXi& T_tags,
    const std::map<std::string, int>& tag_label_map)
{
    // assert dimensions
    assert(V.cols() == 3);
    assert(T.cols() == 4);
    assert(T.rows() == T_tags.rows());

    // extract tag of interest and set map
    if (!tag_label_map.count(m_params.tag_label)) {  // desired tag not found
        std::string existing_tags = "";
        for (const auto& pair : tag_label_map) {
            existing_tags += (" (" + pair.first + ")");
        }
        log_and_throw_error("Tag label '{}' not found in input mesh. [Tags found:{}]", m_params.tag_label, existing_tags);
    }
    for (const auto& pair : tag_label_map) {
        m_label_map[pair.first] = pair.second;
    }
    Eigen::MatrixXi T_tag = T_tags(Eigen::all, m_label_map[m_params.tag_label]);
    m_tags_count = m_label_map.size();

    // initialize connectivity
    init(T);
    assert(check_mesh_connectivity_validity());
    m_vertex_attribute.m_attributes.resize(V.rows());
    m_edge_attribute.m_attributes.resize(6 * T.rows());
    m_face_attribute.m_attributes.resize(4 * T.rows());
    m_tet_attribute.m_attributes.resize(T.rows());

    // initialize vertex coords and labels
    for (size_t i = 0; i < vert_capacity(); i++) {
        m_vertex_attribute[i].m_posf = V.row(i);  // position

        auto tets = get_one_ring_tids_for_vertex(i);  // incident labels
        std::set<int> unique_incident_sep_tags;
        for (const size_t t_id : tets) {
            int tet_tag = T_tag(t_id);

            // if tet has tag of interest (THIS CAN BE GREATLY SPED UP)
            if (std::count(m_params.sep_tags.begin(), m_params.sep_tags.end(), tet_tag) > 0) {
                unique_incident_sep_tags.insert(tet_tag);
            }
        }
        if (unique_incident_sep_tags.size() >= 2) {  // incident to two or more regions to offset.
            m_vertex_attribute[i].label = 1;
        }
    }

    if (m_params.manifold_mode) {  // label non manifold simplices
        int garbage = 0;
    } else {
        // initialize edge labels
        auto edges = get_edges();  // vector<Tuple>
        for (const Tuple e : edges) {
            auto tets = get_incident_tids_for_edge(e);
            std::set<int> unique_incident_tags;
            for (const size_t t_id : tets) {
                int tet_tag = T_tag(t_id);
                
                // if tet has tag of interest
                if (std::count(m_params.sep_tags.begin(), m_params.sep_tags.end(), tet_tag) > 0) {
                    unique_incident_tags.insert(T_tag(t_id));
                }
            }
            if (unique_incident_tags.size() >= 2) {
                m_edge_attribute[e.eid(*this)].label = 1;
            }
        }

        // initialize face labels
        auto faces = get_faces();
        for (const Tuple f : faces) {
            // get tets with current face
            std::vector<size_t> tet_ids;
            tet_ids.push_back(f.tid(*this));
            auto other_tet = switch_tetrahedron(f);
            if (other_tet.has_value()) {
                tet_ids.push_back(other_tet.value().tid(*this));
            }

            std::set<int> unique_incident_tags;
            for (const size_t t_id : tet_ids) {
                int tet_tag = T_tag(t_id);

                // if tet has tag of interest
                if (std::count(m_params.sep_tags.begin(), m_params.sep_tags.end(), tet_tag) > 0) {
                    unique_incident_tags.insert(T_tag(t_id));
                }
            }
            if (unique_incident_tags.size() >= 2) {  // (can be at most 2 for face manifold mesh)
                size_t glob_fid = f.fid(*this);
                m_face_attribute[glob_fid].label = 1;
            }
        }
    }

    // initialize tet tags
    for (size_t i = 0; i < tet_size(); i++) {
        for (int j = 0; j < T_tags.cols(); j++) {
            m_tet_attribute[i].tags.push_back(T_tags(i, j));
        }
    }
}


bool TopoOffsetMesh::is_simplicially_embedded() const {
    int bad_tets = 0;
    auto tets = get_tets();
    for (const Tuple& t : tets) {
        bad_tets += (!tet_is_simp_emb(t));
    }
    if (bad_tets == 0) {
        logger().info("Mesh simplically embedded: TRUE");
        return true;
    } else {
        logger().info("Mesh simplically embedded: FALSE ({} bad tets)", bad_tets);
        return false;
    }
}


bool TopoOffsetMesh::tet_is_simp_emb(const Tuple& t) const {
    if (m_tet_attribute[t.tid(*this)].label == 1) {  // entire tet in input
        return true;
    } else {
        auto vs = oriented_tet_vids(t);
        std::vector<size_t> vs_in;
        for (int i = 0; i < 4; i++) {
            if (m_vertex_attribute[vs[i]].label == 1) {
                vs_in.push_back(vs[i]);
            }
        }
        if (vs_in.size() <= 1) {  // nothing or just one vertex in input
            return true;
        } else if (vs_in.size() == 2) {  // potentially one edge in input
            size_t glob_eid = tuple_from_edge({vs_in[0], vs_in[1]}).eid(*this);
            return (m_edge_attribute[glob_eid].label == 1);
        } else {  // vs_in.size() == 3
            auto [_, glob_fid] = tuple_from_face({vs_in[0], vs_in[1], vs_in[2]});
            return (m_face_attribute[glob_fid].label == 1);
        }
    }
}


void TopoOffsetMesh::simplicial_embedding() {
    // identify necessary tets to split (by vertices)
    std::vector<simplex::Tet> tets_to_split;
    auto tets = get_tets();
    for (const Tuple& tet : tets) {
        auto vs = oriented_tet_vids(tet);
        if (m_tet_attribute[tet.tid(*this)].label != 1) {
            bool to_split = true;
            for (int i = 0; i < 4; i++) {
                size_t v1 = vs[i];
                size_t v2 = vs[(i+1) % 4];
                size_t v3 = vs[(i+2) % 4];
                auto [_, glob_fid] = tuple_from_face({v1, v2, v3});
                if (m_face_attribute[glob_fid].label != 1) {
                    to_split = false;
                }
            }

            if (to_split) {  // tet not in input but all faces are
                tets_to_split.push_back(simplex::Tet(vs[0], vs[1], vs[2], vs[3]));
            }
        }
    }

    // actually split tets
    for (const simplex::Tet& tet : tets_to_split) {
        const auto& vs = tet.vertices();
        Tuple t = tuple_from_vids(vs[0], vs[1], vs[2], vs[3]);
        std::vector<Tuple> garbage;
        split_tet(t, garbage);
    }
    logger().info("Tets split: {}", tets_to_split.size());

    // identify necessary faces to split
    std::vector<simplex::Face> faces_to_split;
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        auto vs = get_face_vids(f);
        if (m_face_attribute[f.fid(*this)].label != 1) {
            bool to_split = true;
            for (int i = 0; i < 3; i++) {
                size_t v1 = vs[i];
                size_t v2 = vs[(i+1) % 3];
                size_t glob_eid = tuple_from_edge({v1, v2}).eid(*this);
                if (m_edge_attribute[glob_eid].label != 1) {
                    to_split = false;
                }
            }

            if (to_split) {
                faces_to_split.push_back(simplex::Face(vs[0], vs[1], vs[2]));
            }
        }
    }

    // actually split faces
    for (const simplex::Face& f : faces_to_split) {
        const auto& vs = f.vertices();
        auto [t, _] = tuple_from_face({vs[0], vs[1], vs[2]});
        std::vector<Tuple> garbage;
        split_face(t, garbage);
    }
    logger().info("Faces split: {}", faces_to_split.size());

    // identify edges to split
    std::vector<simplex::Edge> edges_to_split;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        if (m_edge_attribute[e.eid(*this)].label != 1) {  // edge not in input
            size_t v1_id = e.vid(*this);
            size_t v2_id = switch_vertex(e).vid(*this);
            if ((m_vertex_attribute[v1_id].label == 1) && (m_vertex_attribute[v2_id].label == 1)) {
                edges_to_split.push_back(simplex::Edge(v1_id, v2_id));
            }
        }
    }

    // actually split edges
    for (const simplex::Edge& e : edges_to_split) {
        Tuple t = tuple_from_edge(e.vertices());
        std::vector<Tuple> garbage;
        split_edge(t, garbage);
    }
    logger().info("Edges split: {}", edges_to_split.size());
}


void TopoOffsetMesh::perform_offset() {
    // one vertex cells to split
    std::vector<simplex::Edge> e_to_split;
    std::vector<bool> marked_v;  // all input or split resulting vertices
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        size_t v1 = e.vid(*this);
        size_t v2 = switch_vertex(e).vid(*this);
        if (m_vertex_attribute[v1].label != m_vertex_attribute[v2].label) {
            e_to_split.push_back(simplex::Edge(v1, v2));
        }
    }

    // mark labeled vertices (ASSUMES MESH CONSOLIDATED)
    for (int v_id = 0; v_id < vertex_size(); v_id++) {
        marked_v.push_back(m_vertex_attribute[v_id].label == 1);
    }

    // split edges and mark subsequent verts
    std::vector<Tuple> garbage;
    for (const simplex::Edge& e : e_to_split) {
        Tuple t = tuple_from_edge(e.vertices());
        split_edge(t, garbage);
        garbage.clear();
        marked_v.push_back(true);
    }

    // mark all offset tets as such
    auto tets = get_tets();
    for (const Tuple& t : tets) {
        auto vs = oriented_tet_vids(t);
        bool is_offset = true;
        for (const size_t v : vs) {
            if (!marked_v[v]) {  // vert not marked
                is_offset = false;
            }
        }

        if (is_offset) {  // mark tet as offset (TODO: propogate to children simplices)
            m_tet_attribute[t.tid(*this)].label = 2;
            m_tet_attribute[t.tid(*this)].tags[m_label_map[m_params.tag_label]] = m_params.fill_tag;
        }
    }
}


bool TopoOffsetMesh::invariants(const std::vector<Tuple>& tets)
{
    return true;
}


void TopoOffsetMesh::write_msh(std::string file) {
    logger().info("Write {}", file);
    consolidate_mesh();

    wmtk::MshData msh;

    const auto& vtx = get_vertices();
    msh.add_tet_vertices(vtx.size(), [&](size_t k) {
        auto i = vtx[k].vid(*this);
        return m_vertex_attribute[i].m_posf;
    });

    const auto& tets = get_tets();
    msh.add_tets(tets.size(), [&](size_t k) {
        auto i = tets[k].tid(*this);
        auto vs = oriented_tet_vertices(tets[k]);
        std::array<size_t, 4> data;
        for (int j = 0; j < 4; j++) {
            data[j] = vs[j].vid(*this);
            assert(data[j] < vtx.size());
        }
        return data;
    });

    for (const auto& pair : m_label_map) {
        int ind = pair.second;
        msh.add_tet_attribute<1>(pair.first, [&](size_t i) {
            return m_tet_attribute[i].tags[ind];
        });
    }

    msh.save(file, true);
}


void TopoOffsetMesh::write_input_complex(const std::string& path) {
    logger().info("Write {}", path);

    std::vector<int> vid_map(vertex_size(), -1);  // vid_map[i] gives new vertex id for old id 'i'
    std::vector< std::vector<int> > cells;

    // extract required vertices and populate id map
    std::vector<Eigen::Vector3d> verts_to_offset;  // vertices to offset
    for (size_t i = 0; i < vertex_size(); i++) {
        if (m_vertex_attribute[i].label == 1) {
            verts_to_offset.push_back(m_vertex_attribute[i].m_posf);
            vid_map[i] = verts_to_offset.size() - 1;
        }
    }
    Eigen::MatrixXd V(verts_to_offset.size(), 3);
    for (int i = 0; i < V.rows(); i++) {
        V.row(i) = verts_to_offset[i];
    }

    // get all boundary edges
    auto edges = get_edges();
    for (const Tuple e : edges) {
        if (m_edge_attribute[e.eid(*this)].label == 1) {
            std::vector<int> curr_e;
            curr_e.push_back(vid_map[e.vid(*this)]);
            curr_e.push_back(vid_map[switch_vertex(e).vid(*this)]);
            cells.push_back(curr_e);
        }
    }

    // get all boundary faces
    auto faces = get_faces();
    for (const Tuple f : faces) {
        if (m_face_attribute[f.fid(*this)].label == 1) {
            std::vector<int> curr_f;
            curr_f.push_back(vid_map[f.vid(*this)]);
            Tuple f1 = switch_vertex(f);
            curr_f.push_back(vid_map[f1.vid(*this)]);
            Tuple f2 = switch_vertex(switch_edge(f1));
            curr_f.push_back(vid_map[f2.vid(*this)]);
            cells.push_back(curr_f);
        }
    }


    // get all 'boundary' tets (only matters for manifold extraction)
    bool flag = false;
    for (size_t i = 0; i < tet_size(); i++) {
        if (m_tet_attribute[i].label == 1) {
            flag = true;
            auto vids = oriented_tet_vids(i);
            std::vector<int> curr_t;
            for (const size_t vid : vids) {
                curr_t.push_back(vid_map[vid]);
            }
        }
    }
    if (flag) { logger().warn("One or more tet included in complex to offset."); };

    // output
    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();
    writer->write_mesh(path + ".vtu", V, cells, true, false);
}


void TopoOffsetMesh::write_vtu(const std::string& path)
{
    consolidate_mesh();
    const std::string out_path = path + ".vtu";
    logger().info("Write {}", out_path);
    const auto& vs = get_vertices();
    const auto& tets = get_tets();

    Eigen::MatrixXd V(vert_capacity(), 3);
    Eigen::MatrixXi T(tet_capacity(), 4);

    V.setZero();
    T.setZero();

    std::vector<MatrixXi> tags(m_tags_count, MatrixXi(tet_capacity(), 1));

    int index = 0;
    for (const Tuple& t : tets) {
        size_t tid = t.tid(*this);
        for (const auto& pair : m_label_map) {
            tags[pair.second](index, 0) = m_tet_attribute[tid].tags[pair.second];
        }

        const auto& loc_vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            T(index, j) = loc_vs[j].vid(*this);
        }
        index++;
    }

    for (const Tuple& v : vs) {
        const size_t vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_posf;
    }

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();

    for (const auto& pair : m_label_map) {
        // logger().info("{} {}", pair.first, pair.second);
        writer->add_cell_field(pair.first, tags[pair.second].cast<double>());
    }
    writer->write_mesh(out_path, V, T);
}

} // namespace wmtk::components::topological_offset