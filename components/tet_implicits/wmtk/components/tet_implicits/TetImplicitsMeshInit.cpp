#include "TetImplicitsMesh.h"

#include <igl/remove_unreferenced.h>

namespace wmtk::components::tet_implicits {

void TetImplicitsMesh::init_from_image(const MatrixXd& V, const MatrixXi& T, const MatrixXi& T_tags)
{
    assert(V.cols() == 3);
    assert(T.cols() == 4);
    assert(T_tags.rows() == T.rows());

    init(T);

    assert(check_mesh_connectivity_validity());

    m_tags_count = T_tags.cols();

    m_vertex_attribute.m_attributes.resize(V.rows());
    m_tet_attribute.m_attributes.resize(T.rows());
    // m_face_attribute.m_attributes.resize(T.rows() * 4);

    for (int i = 0; i < vert_capacity(); i++) {
        m_vertex_attribute[i].m_posf = V.row(i);
    }

    // sanity check
    for (const Tuple& t : get_tets()) {
        if (is_inverted(t)) {
            log_and_throw_error("Inverted tet in the input!");
        }
    }
    logger().info("Sanity check passed; no inverted tets found");

    // add tags
    for (size_t i = 0; i < T_tags.rows(); ++i) {
        m_tet_attribute[i].tags.resize(m_tags_count);
        for (size_t j = 0; j < m_tags_count; ++j) {
            m_tet_attribute[i].tags[j] = T_tags(i, j);
        }
    }

    init_bvhs();

    // init qualities
    for_each_tetra(
        [this](const Tuple& t) { m_tet_attribute[t.tid(*this)].m_quality = get_quality(t); });
}

void TetImplicitsMesh::init_bvhs()
{
    const auto faces = get_faces();
    logger().info("faces size: {}", faces.size());

    // std::set<std::pair<int64_t, int64_t>> all_tags;
    // for (const Tuple& t : get_tets()) {
    //     const size_t tid = t.tid(*this);
    //     for (size_t i = 0; i < m_tags_count; ++i) {
    //         std::pair<int64_t, int64_t> tag;
    //         tag.first = i;
    //         tag.second = m_tet_attribute[tid].tags[i];
    //         all_tags.insert(tag);
    //     }
    // }

    std::map<std::pair<int64_t, int64_t>, std::vector<Vector3i>> F_vectors;

    // std::vector<Eigen::Vector3i> tempF;
    // tag surface faces and vertices
    for (const Tuple& f : faces) {
        SmartTuple ff(*this, f);

        const auto t_opp = ff.switch_tetrahedron();
        if (!t_opp) {
            continue;
        }

        const size_t v0 = ff.vid();
        const size_t v1 = ff.switch_vertex().vid();
        const size_t v2 = ff.switch_edge().switch_vertex().vid();

        for (int64_t j = 0; j < m_tags_count; ++j) {
            const int64_t tag0 = m_tet_attribute[ff.tid()].tags[j];
            const int64_t tag1 = m_tet_attribute[t_opp.value().tid()].tags[j];

            if (tag0 == tag1) {
                continue;
            }

            // has two different tags --> add to BVHs
            F_vectors[std::make_pair(j, tag0)].emplace_back(v0, v1, v2);
            F_vectors[std::make_pair(j, tag1)].emplace_back(v0, v1, v2);
        }
    }

    MatrixXd VV;
    const auto verts = get_vertices();
    VV.resize(verts.size(), 3);
    for (size_t i = 0; i < verts.size(); ++i) {
        VV.row(i) = m_vertex_attribute[verts[i].vid(*this)].m_posf;
    }

    for (const auto& [tag_id, F] : F_vectors) {
        MatrixXi FF;
        FF.resize(F.size(), 3);
        for (size_t i = 0; i < F.size(); ++i) {
            FF.row(i) = F[i];
        }

        MatrixXd NV;
        MatrixXi NF;
        VectorXi I;
        igl::remove_unreferenced(VV, FF, NV, NF, I);

        auto& bvh = m_bvh[tag_id];
        bvh = std::make_shared<SimpleBVH::BVH>();
        bvh->init(NV, NF, 0);
    }
}

} // namespace wmtk::components::tet_implicits