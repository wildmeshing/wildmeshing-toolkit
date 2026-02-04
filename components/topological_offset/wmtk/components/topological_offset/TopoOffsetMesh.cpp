
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

#include <cmath>
#include <limits>


// namespace {
// static int debug_print_counter = 0;
// }

namespace wmtk::components::topological_offset {


VertexAttributes::VertexAttributes(const Vector3d& p)
    : m_posf(p)
{}


// assumes tag has been found. won't be called otherwise
void TopoOffsetMesh::init_from_image(
    const MatrixXd& V,
    const MatrixXi& T,
    const MatrixXd& T_tags,
    const std::vector<std::string>& all_tag_names)
{
    // assert dimensions
    assert(V.cols() == 3);
    assert(T.cols() == 4);
    assert(T.rows() == T_tag.rows());

    // save tags info to mesh (assumes desired tag is in tag names found)
    auto it = std::find(
        all_tag_names.begin(),
        all_tag_names.end(),
        m_params.tag_name); // note: this is const iterator
    m_toi_ind = std::distance(all_tag_names.begin(), it);
    m_tags_count = all_tag_names.size();
    for (const auto& name : all_tag_names) {
        m_all_tag_names.push_back(name);
    }

    // initialize connectivity
    init(T);
    assert(check_mesh_connectivity_validity());
    m_vertex_attribute.m_attributes.resize(V.rows());
    m_edge_attribute.m_attributes.resize(6 * T.rows());
    m_face_attribute.m_attributes.resize(4 * T.rows());
    m_tet_attribute.m_attributes.resize(T.rows());

    MatrixXi T_tag = T_tags.col(m_toi_ind).cast<int>(); // toi vals as ints

    // propogate labels to tets
    auto tets = get_tets();
    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);
        for (int i = 0; i < m_tags_count; i++) {
            m_tet_attribute[t_id].tags.push_back(T_tags(t_id, i));
        }
    }

    // label boundary faces
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        // find unique incident tet tags
        std::set<int> nb_tags;
        nb_tags.insert(T_tag(f.tid(*this), 0));
        auto other = f.switch_tetrahedron(*this);
        if (other) {
            nb_tags.insert(T_tag(other.value().tid(*this), 0));
        }

        // count number of unique incident tet tags in sep_tags
        if (count_sep_tags(nb_tags) >= 2) { // can be max of two for face-manifold mesh
            size_t f_id = f.fid(*this);
            m_face_attribute[f_id].label = 1;

            // label children edges and vertices as input
            m_edge_attribute[f.eid(*this)].label = 1;
            m_edge_attribute[f.switch_edge(*this).eid(*this)].label = 1;
            m_edge_attribute[f.switch_vertex(*this).switch_edge(*this).eid(*this)].label = 1;
            m_vertex_attribute[f.vid(*this)].label = 1;
            m_vertex_attribute[f.switch_vertex(*this).vid(*this)].label = 1;
            m_vertex_attribute[f.switch_edge(*this).switch_vertex(*this).vid(*this)].label = 1;
        }
    }

    // label boundary edges. NOTE: ideally could skip over edges already labelled via parent face
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        // find unique incident tet tags
        std::set<int> nb_tags;
        auto inc_t_ids = get_incident_tids_for_edge(e);
        for (const size_t& t_id : inc_t_ids) {
            nb_tags.insert(T_tag(t_id, 0));
        }

        // count number of unique incident tet tags in sep_tags
        if (count_sep_tags(nb_tags) >= 2) {
            size_t e_id = e.eid(*this);
            m_edge_attribute[e_id].label = 1;

            // label children verts as input
            m_vertex_attribute[e.vid(*this)].label = 1;
            m_vertex_attribute[e.switch_vertex(*this).vid(*this)].label = 1;
        }
    }

    // label boundary vertices (and set coords)
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        // set position
        size_t v_id = v.vid(*this);
        m_vertex_attribute[v_id].m_posf = V.row(v_id);

        // check if vertex already labeled (cheap, might as well do)
        if (m_vertex_attribute[v_id].label == 1) {
            continue;
        }

        // collect incident tet tags
        auto inc_t_ids = get_one_ring_tids_for_vertex(v_id);
        std::set<int> nb_tags;
        for (const size_t& t_id : inc_t_ids) {
            nb_tags.insert(T_tag(t_id, 0));
        }

        // determine if on boundary of sep tags
        if (count_sep_tags(nb_tags) >= 2) {
            m_vertex_attribute[v_id].label = 1;
        }
    }
}


bool TopoOffsetMesh::is_simplicially_embedded() const
{
    int bad_tets = 0;
    auto tets = get_tets();
    for (const Tuple& t : tets) {
        bad_tets += (!tet_is_simp_emb(t));
    }
    if (bad_tets == 0) {
        logger().info("\tMesh simplicially embedded: TRUE");
        return true;
    } else {
        logger().info("\tMesh simplicially embedded: FALSE ({} bad tets)", bad_tets);
        return false;
    }
}


bool TopoOffsetMesh::tet_is_simp_emb(const Tuple& t) const
{
    auto vs = oriented_tet_vids(t);
    std::vector<size_t> vs_in;
    for (int i = 0; i < 4; i++) {
        if (m_vertex_attribute[vs[i]].label == 1) {
            vs_in.push_back(vs[i]);
        }
    }
    if (vs_in.size() <= 1) { // nothing or just one vertex in input
        return true;
    } else if (vs_in.size() == 2) { // potentially one edge in input
        size_t glob_eid = tuple_from_edge({vs_in[0], vs_in[1]}).eid(*this);
        return (m_edge_attribute[glob_eid].label == 1);
    } else if (vs_in.size() == 3) { // potentially one face in input
        auto [_, glob_fid] = tuple_from_face({vs_in[0], vs_in[1], vs_in[2]});
        return (m_face_attribute[glob_fid].label == 1);
    } else { // all four verts in complex, can't be simplicially embedded
        return false;
    }
}


void TopoOffsetMesh::simplicial_embedding()
{
    // identify necessary tets to split (by vertices)
    std::vector<simplex::Tet> tets_to_split;
    auto tets = get_tets();
    for (const Tuple& tet : tets) {
        auto vs = oriented_tet_vids(tet);
        if (m_tet_attribute[tet.tid(*this)].label != 1) {
            bool to_split = true;
            for (int i = 0; i < 4; i++) {
                size_t v1 = vs[i];
                size_t v2 = vs[(i + 1) % 4];
                size_t v3 = vs[(i + 2) % 4];
                auto [_, glob_fid] = tuple_from_face({v1, v2, v3});
                if (m_face_attribute[glob_fid].label != 1) {
                    to_split = false;
                }
            }

            if (to_split) { // tet not in input but all faces are
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
    logger().info("\tTets split: {}", tets_to_split.size());

    // identify necessary faces to split
    std::vector<simplex::Face> faces_to_split;
    auto faces = get_faces();
    for (const Tuple& f : faces) {
        auto vs = get_face_vids(f);
        if (m_face_attribute[f.fid(*this)].label != 1) {
            bool to_split = true;
            for (int i = 0; i < 3; i++) {
                size_t v1 = vs[i];
                size_t v2 = vs[(i + 1) % 3];
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
    logger().info("\tFaces split: {}", faces_to_split.size());

    // identify edges to split
    std::vector<simplex::Edge> edges_to_split;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        if (m_edge_attribute[e.eid(*this)].label != 1) { // edge not in input
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
    logger().info("\tEdges split: {}", edges_to_split.size());
}


void TopoOffsetMesh::perform_offset()
{
    // mark edges to split
    std::vector<simplex::Edge> e_to_split;
    auto edges = get_edges();
    for (const Tuple& e : edges) {
        size_t v1 = e.vid(*this);
        size_t v2 = e.switch_vertex(*this).vid(*this);
        if ((m_vertex_attribute[v1].label + m_vertex_attribute[v2].label) == 1) {
            e_to_split.push_back(simplex::Edge(v1, v2));
        }
    }

    // sort edges (split longest first), should give better output mesh quality
    logger().info("\tSorting edges by length...");
    sort_edges_by_length(e_to_split);

    // actually split edges
    std::vector<Tuple> new_edges;
    for (const simplex::Edge& e : e_to_split) {
        // split edge
        new_edges.clear();
        Tuple t = tuple_from_edge(e.vertices());
        split_edge(t, new_edges);
    }

    // mark all offset tets (all tets with any vertex labeled 1)
    auto tets = get_tets();
    for (const Tuple& tet : tets) {
        auto vs = oriented_tet_vids(tet);

        bool in_offset = false;
        for (size_t v : vs) { // vertices
            if (m_vertex_attribute[v].label == 1) {
                in_offset = true;
            }
        }

        if (in_offset) {
            m_tet_attribute[tet.tid(*this)].label = 2; // not actually used anywhere
            m_tet_attribute[tet.tid(*this)].tags[m_toi_ind] = m_params.offset_tag_val;
        }
    }
}


bool TopoOffsetMesh::invariants(const std::vector<Tuple>& tets)
{
    igl::predicates::exactinit();
    for (const Tuple& t : tets) {
        auto vs = oriented_tet_vids(t);
        auto res = igl::predicates::orient3d(
            m_vertex_attribute[vs[0]].m_posf,
            m_vertex_attribute[vs[1]].m_posf,
            m_vertex_attribute[vs[2]].m_posf,
            m_vertex_attribute[vs[3]].m_posf);

        if (res != igl::predicates::Orientation::NEGATIVE) {
            return false;
        }
    }
    return true;
}


void TopoOffsetMesh::write_input_complex(const std::string& path)
{
    logger().info("Write {}.vtu", path);

    std::vector<int> vid_map(vertex_size(),
                             -1); // vid_map[i] gives new vertex id for old id 'i'
    std::vector<std::vector<int>> cells;

    // extract required vertices and populate id map
    std::vector<Eigen::Vector3d> verts_to_offset; // vertices to offset
    auto verts = get_vertices();
    for (const Tuple& v : verts) {
        size_t i = v.vid(*this);
        if (m_vertex_attribute[i].label == 1) {
            verts_to_offset.push_back(m_vertex_attribute[i].m_posf);
            vid_map[i] = verts_to_offset.size() - 1;
        }
    }
    Eigen::MatrixXd V(verts_to_offset.size(), 3);
    for (int i = 0; i < V.rows(); i++) {
        V.row(i) = verts_to_offset[i];
    }

    // get all offset input edges
    auto edges = get_edges();
    for (const Tuple e : edges) {
        if (m_edge_attribute[e.eid(*this)].label == 1) {
            std::vector<int> curr_e;
            curr_e.push_back(vid_map[e.vid(*this)]);
            curr_e.push_back(vid_map[switch_vertex(e).vid(*this)]);
            cells.push_back(curr_e);
        }
    }

    // get all offset input faces
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

    // get all offset input tets ( this should never happen? )
    bool flag = false;
    int num_tets = tet_size();
    for (size_t i = 0; i < num_tets; i++) {
        if (m_tet_attribute[i].label == 1) {
            flag = true;
            auto vids = oriented_tet_vids(i);
            std::vector<int> curr_t;
            for (const size_t vid : vids) {
                curr_t.push_back(vid_map[vid]);
            }
        }
    }
    if (flag) {
        logger().warn("One or more tet included in complex to offset.");
    };

    // output
    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();
    writer->write_mesh(path + ".vtu", V, cells, true, false);
}


void TopoOffsetMesh::write_vtu(const std::string& path)
{
    const std::string out_path = path + ".vtu";
    logger().info("Write {}", out_path);

    consolidate_mesh();
    const auto& vs = get_vertices();
    const auto& tets = get_tets();

    Eigen::MatrixXd V(vert_capacity(), 3);
    Eigen::MatrixXi T(tet_capacity(), 4);

    V.setZero();
    T.setZero();

    std::vector<MatrixXd> tags(m_tags_count, MatrixXd(tet_capacity(), 1));

    for (const Tuple& t : tets) {
        size_t t_id = t.tid(*this);

        // set tet tags
        for (int i = 0; i < m_tags_count; i++) {
            tags[i](t_id, 0) = m_tet_attribute[t_id].tags[i];
        }

        // set tet verts
        const auto& loc_vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            T(t.tid(*this), j) = loc_vs[j].vid(*this);
        }
    }

    for (const Tuple& v : vs) {
        const size_t vid = v.vid(*this);
        V.row(vid) = m_vertex_attribute[vid].m_posf;
    }

    std::shared_ptr<paraviewo::ParaviewWriter> writer;
    writer = std::make_shared<paraviewo::VTUWriter>();

    int index = 0;
    for (const std::string& tag_name : m_all_tag_names) {
        writer->add_cell_field(tag_name, tags[index]);
        index++;
    }

    writer->write_mesh(out_path, V, T);
}


void TopoOffsetMesh::write_msh(const std::string& file)
{
    logger().info("Write {}.msh", file);
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

    for (size_t j = 0; j < m_tags_count; ++j) {
        msh.add_tet_attribute<1>(m_all_tag_names[j], [&](size_t i) {
            return m_tet_attribute[i].tags[j];
        });
    }

    msh.save(file + ".msh", true);
}


} // namespace wmtk::components::topological_offset