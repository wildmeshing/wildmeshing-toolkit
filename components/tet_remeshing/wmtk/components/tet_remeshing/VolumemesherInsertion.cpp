#include <fstream>
#include <set>
#include "TetRemeshingMesh.h"

namespace wmtk::components::tet_remeshing {

void TetRemeshingMesh::init_from_image(const MatrixXd& V, const MatrixXi& T, const MatrixXi& T_tags)
{
    assert(V.cols() == 3);
    assert(T.cols() == 4);
    assert(T_tags.rows() == T.rows());

    init(T);

    assert(check_mesh_connectivity_validity());

    m_tags_count = T_tags.cols();

    m_vertex_attribute.m_attributes.resize(V.rows());
    m_tet_attribute.m_attributes.resize(T.rows());
    m_face_attribute.m_attributes.resize(T.rows() * 4);

    for (int i = 0; i < vert_capacity(); i++) {
        m_vertex_attribute[i].m_posf = V.row(i);
    }

    // sanity check
    for (const Tuple& t : get_tets()) {
        if (is_inverted_f(t)) {
            log_and_throw_error("Inverted tet in the input!");
        }
    }

    // add tags
    for (size_t i = 0; i < T_tags.rows(); ++i) {
        m_tet_attribute[i].tags.resize(m_tags_count);
        for (size_t j = 0; j < m_tags_count; ++j) {
            m_tet_attribute[i].tags[j] = T_tags(i, j);
        }
    }

    init_surfaces_and_boundaries();

    // init qualities
    for_each_tetra(
        [this](const Tuple& t) { m_tet_attribute[t.tid(*this)].m_quality = get_quality(t); });
}

void TetRemeshingMesh::init_surfaces_and_boundaries()
{
    const auto faces = get_faces();
    std::cout << "faces size: " << faces.size() << std::endl;

    // tag surface faces and vertices
    std::vector<Eigen::Vector3i> tempF;
    for (const Tuple& f : faces) {
        SmartTuple ff(*this, f);

        const auto t_opp = ff.switch_tetrahedron();
        if (!t_opp) {
            continue;
        }

        bool has_two_tags = false;

        for (size_t j = 0; j < m_tags_count; ++j) {
            const int64_t tag0 = m_tet_attribute[ff.tid()].tags[j];
            const int64_t tag1 = m_tet_attribute[t_opp.value().tid()].tags[j];

            if (tag0 != tag1) {
                has_two_tags = true;
                break;
            }
        }

        if (!has_two_tags) {
            continue;
        }

        m_face_attribute[ff.fid()].m_is_surface_fs = 1;

        const size_t v1 = ff.vid();
        const size_t v2 = ff.switch_vertex().vid();
        const size_t v3 = ff.switch_edge().switch_vertex().vid();
        m_vertex_attribute[v1].m_is_on_surface = true;
        m_vertex_attribute[v2].m_is_on_surface = true;
        m_vertex_attribute[v3].m_is_on_surface = true;

        tempF.emplace_back(v1, v2, v3);
    }

    if (!m_envelope) {
        logger().info("Init envelope from tet tags");
        // build envelopes
        std::vector<Eigen::Vector3d> tempV(vert_capacity());
        for (int i = 0; i < vert_capacity(); i++) {
            tempV[i] = m_vertex_attribute[i].m_posf;
        }

        m_V_envelope = tempV;
        m_F_envelope = tempF;
        m_envelope = std::make_shared<ExactEnvelope>();
        m_envelope->init(m_V_envelope, m_F_envelope, m_envelope_eps);
        triangles_tree = std::make_shared<SampleEnvelope>();
        triangles_tree->init(m_V_envelope, m_F_envelope, m_envelope_eps);
    }

    // All surface faces must be inside the envelope
    {
        logger().info("Envelope sanity check");
        const auto surf_faces = get_faces_by_condition([](auto& f) { return f.m_is_surface_fs; });
        for (const auto& verts : surf_faces) {
            const auto& p0 = m_vertex_attribute[verts[0]].m_posf;
            const auto& p1 = m_vertex_attribute[verts[1]].m_posf;
            const auto& p2 = m_vertex_attribute[verts[2]].m_posf;
            if (m_envelope->is_outside({{p0, p1, p2}})) {
                log_and_throw_error("Face {} is outside!", verts);
            }
        }
        logger().info("Envelope sanity check done");
    }

    // track bounding box
    for (size_t i = 0; i < faces.size(); i++) {
        const auto vs = get_face_vertices(faces[i]);
        std::array<size_t, 3> vids = {{vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
        int on_bbox = -1;
        for (int k = 0; k < 3; k++) {
            if (m_vertex_attribute[vids[0]].m_posf[k] == m_params.box_min[k] &&
                m_vertex_attribute[vids[1]].m_posf[k] == m_params.box_min[k] &&
                m_vertex_attribute[vids[2]].m_posf[k] == m_params.box_min[k]) {
                on_bbox = k * 2;
                break;
            }
            if (m_vertex_attribute[vids[0]].m_posf[k] == m_params.box_max[k] &&
                m_vertex_attribute[vids[1]].m_posf[k] == m_params.box_max[k] &&
                m_vertex_attribute[vids[2]].m_posf[k] == m_params.box_max[k]) {
                on_bbox = k * 2 + 1;
                break;
            }
        }
        if (on_bbox < 0) {
            continue;
        }
        assert(!faces[i].switch_tetrahedron(*this)); // face must be on boundary

        const size_t fid = faces[i].fid(*this);
        m_face_attribute[fid].m_is_bbox_fs = on_bbox;

        for (const size_t vid : vids) {
            m_vertex_attribute[vid].on_bbox_faces.push_back(on_bbox);
        }
    }

    for_each_vertex(
        [&](auto& v) { wmtk::vector_unique(m_vertex_attribute[v.vid(*this)].on_bbox_faces); });

    // track open boundaries
    find_open_boundary();
}

void TetRemeshingMesh::find_open_boundary()
{
    const auto faces = get_faces();
    std::vector<int> edge_on_open_boundary(6 * tet_capacity(), 0);

    // for open boundary envelope
    std::vector<Eigen::Vector3d> v_posf(vert_capacity());
    std::vector<Eigen::Vector3i> open_boundaries;

    for (size_t i = 0; i < vert_capacity(); i++) {
        v_posf[i] = m_vertex_attribute[i].m_posf;
    }

    for (const Tuple& f : faces) {
        const SmartTuple ff(*this, f);
        const size_t fid = ff.fid();
        if (!m_face_attribute[fid].m_is_surface_fs) {
            continue;
        }
        size_t eid1 = ff.eid();
        size_t eid2 = ff.switch_edge().eid();
        size_t eid3 = ff.switch_vertex().switch_edge().eid();

        edge_on_open_boundary[eid1]++;
        edge_on_open_boundary[eid2]++;
        edge_on_open_boundary[eid3]++;
    }

    const auto edges = get_edges();
    for (const Tuple& e : edges) {
        if (edge_on_open_boundary[e.eid(*this)] != 1) {
            continue;
        }
        size_t v1 = e.vid(*this);
        size_t v2 = e.switch_vertex(*this).vid(*this);
        m_vertex_attribute[v1].m_is_on_open_boundary = true;
        m_vertex_attribute[v2].m_is_on_open_boundary = true;
        open_boundaries.emplace_back(v1, v2, v1); // degenerate triangle to mimic the edge
    }

    wmtk::logger().info("open boundary num: {}", open_boundaries.size());

    if (open_boundaries.size() == 0) {
        return;
    }

    // init open boundary envelope
    m_open_boundary_envelope.init(v_posf, open_boundaries, m_params.epsr * m_params.diag_l / 2.0);
    boundaries_tree.init(v_posf, open_boundaries, m_params.epsr * m_params.diag_l / 2.0);
}

bool TetRemeshingMesh::is_open_boundary_edge(const Tuple& e)
{
    size_t v1 = e.vid(*this);
    size_t v2 = e.switch_vertex(*this).vid(*this);
    if (!m_vertex_attribute[v1].m_is_on_open_boundary ||
        !m_vertex_attribute[v2].m_is_on_open_boundary)
        return false;

    /*
     * This code is not reliable. If the envelope is chosen too large, elements could be reported as
     * boundary even though they aren't. Especially, when there are sliver triangles that are barely
     * not inverted.
     */

    return !m_open_boundary_envelope.is_outside(
        {{m_vertex_attribute[v1].m_posf,
          m_vertex_attribute[v2].m_posf,
          m_vertex_attribute[v1].m_posf}});
}

bool TetRemeshingMesh::is_open_boundary_edge(const std::array<size_t, 2>& e)
{
    size_t v1 = e[0];
    size_t v2 = e[1];
    if (!m_vertex_attribute[v1].m_is_on_open_boundary ||
        !m_vertex_attribute[v2].m_is_on_open_boundary)
        return false;

    return !m_open_boundary_envelope.is_outside(
        {{m_vertex_attribute[v1].m_posf,
          m_vertex_attribute[v2].m_posf,
          m_vertex_attribute[v1].m_posf}});
}

} // namespace wmtk::components::tet_remeshing