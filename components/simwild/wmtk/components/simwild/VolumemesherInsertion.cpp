#include <bitset>
#include <fstream>
#include <set>
#include <wmtk/utils/Delaunay.hpp>
#include <wmtk/utils/predicates.hpp>
#include "SimWildMesh.h"

namespace wmtk::components::simwild {

void SimWildMesh::init_from_image(
    const MatrixXr& V,
    const MatrixXi& T,
    const MatrixSi& T_tags,
    const std::vector<std::string>& tag_names)
{
    assert(V.cols() == 3);
    assert(T.cols() == 4);
    assert(T_tags.rows() == T.rows());

    init(T);

    assert(check_mesh_connectivity_validity());

    m_tags_count = T_tags.cols();

    m_vertex_attribute.resize(V.rows());
    m_tet_attribute.resize(T.rows());
    m_face_attribute.resize(T.rows() * 4);

    auto& VA = m_vertex_attribute;

    for (int i = 0; i < vert_capacity(); i++) {
        VA[i].m_pos = V.row(i);
        VA[i].m_posf = to_double(VA[i].m_pos);
    }

    if (m_params.perform_sanity_checks) {
        for_each_tetra([this](const Tuple& t) {
            if (is_inverted(t)) {
                log_and_throw_error("Inverted tet in the input!");
            }
        });
    }

    // add tags
    for (size_t i = 0; i < (size_t)T_tags.rows(); ++i) {
        for (size_t j = 0; j < m_tags_count; ++j) {
            if (T_tags.coeff(i, j) != 0) {
                m_tet_attribute[i].tags.insert(j);
            }
        }
    }

    // add tag names
    for (size_t i = 0; i < tag_names.size(); ++i) {
        m_tag_id_to_name[i] = tag_names[i];
        m_tag_name_to_id[tag_names[i]] = i;
    }

    init_surfaces_and_boundaries();

    if (m_params.preserve_topology) {
        init_vertex_order();
    }

    if (m_params.perform_sanity_checks) {
        logger().info("Check tet orientation after initialization...");
        for_each_tetra([this](const Tuple& t) {
            if (is_inverted(t)) {
                log_and_throw_error("Inverted tet in input!");
            }
        });
        logger().info("done");
    }

    // rounding
    {
        std::vector<char> is_direct_point(vert_capacity(), 0);
        // Per-vertex, independent: each i writes only its own m_pos/m_posf/
        // m_is_rounded/is_direct_point; the rational->double conversion and the
        // exact comparison read only local data.
        threading::parallel_for(
            threading::range(0, vert_capacity()),
            [&](const threading::range& range) {
                for (size_t i = range.begin(); i < range.end(); ++i) {
                    const Vector3r& r = VA[i].m_pos;
                    const Vector3r r_from_d = to_rational(VA[i].m_posf);
                    is_direct_point[i] = (r_from_d == r);
                    VA[i].m_is_rounded = is_direct_point[i];
                }
            },
            NUM_THREADS);

        // Round the indirect vertices in parallel. Rounding snaps m_pos to its double
        // and may not invert any incident tet, so it is not independent across adjacent
        // vertices and the old code did it serially (round() per vertex, an exact-
        // rational inversion check over each vertex's incident tets). Instead, snap all
        // indirect vertices at once and then revalidate/revert the few conflicts:
        //   1. snap every not-yet-rounded (indirect) vertex to its double (parallel);
        //   2. scan all tets (parallel) -- any inverted tet must contain an indirect
        //      vertex (a direct vertex never moves, so cannot cause an inversion), so
        //      revert those indirect vertices to their exact rational coordinate;
        //   3. repeat until no tet is inverted.
        // This terminates: every non-empty pass un-rounds at least one vertex, and the
        // all-rational mesh is valid. Direct points are skipped throughout.
        for_each_vertex([&](const Tuple& v) {
            const size_t i = v.vid(*this);
            if (!VA[i].m_is_rounded) {
                VA[i].m_pos = to_rational(VA[i].m_posf);
                VA[i].m_is_rounded = true;
            }
        });

        while (true) {
            // A flag per vertex rather than a shared appended list: a vertex is reachable
            // from several tets, so the old version took a lock per push and then pushed the
            // same vid repeatedly. Every write here stores the same value to a slot indexed
            // by vid, so the threads cannot disagree -- relaxed is enough, and the duplicates
            // collapse for free.
            std::vector<std::atomic<uint8_t>> to_revert(vert_capacity());
            for (auto& f : to_revert) {
                f.store(0, std::memory_order_relaxed);
            }

            std::atomic<bool> any{false};
            for_each_tetra([&](const Tuple& t) {
                if (is_inverted(t)) {
                    for (const size_t vid : oriented_tet_vids(t)) {
                        if (!is_direct_point[vid] && m_vertex_attribute[vid].m_is_rounded) {
                            to_revert[vid].store(1, std::memory_order_relaxed);
                            any.store(true, std::memory_order_relaxed);
                        }
                    }
                }
            });
            if (!any.load(std::memory_order_relaxed)) {
                break;
            }
            for (size_t vid = 0; vid < to_revert.size(); ++vid) {
                if (!to_revert[vid].load(std::memory_order_relaxed)) {
                    continue;
                }
                if (m_vertex_attribute[vid].m_is_rounded) {
                    m_vertex_attribute[vid].m_pos = V.row(vid);
                    m_vertex_attribute[vid].m_is_rounded = false;
                    m_all_rounded.store(false, std::memory_order_relaxed);
                }
            }
        }

        const auto vertices = get_vertices();
        size_t cnt_round_parallel = 0;
        for (const Tuple& v : vertices) {
            if (VA[v.vid(*this)].m_is_rounded) {
                ++cnt_round_parallel;
            }
        }

        // Final serial sweep: the parallel batch reverts a vertex whenever it lies in
        // any inverted tet, which can over-revert (a vertex may round fine once its
        // neighbours are committed). Retry the leftovers one at a time -- round() is a
        // no-op for the already-rounded majority, so this only pays for the few that
        // remain, and recovers vertices the batch conservatively reverted.
        for (const Tuple& v : vertices) {
            round(v);
        }

        size_t cnt_round = 0;
        for (const Tuple& v : vertices) {
            if (VA[v.vid(*this)].m_is_rounded) {
                ++cnt_round;
            }
        }

        logger().info(
            "Rounded vertices {}/{} (parallel {}, serial recovered {})",
            cnt_round,
            vertices.size(),
            cnt_round_parallel,
            cnt_round - cnt_round_parallel);
    }

    // init qualities
    for_each_tetra(
        [this](const Tuple& t) { m_tet_attribute[t.tid(*this)].m_quality = get_quality(t); });

    if (m_params.perform_sanity_checks) {
        logger().info("Check tet orientation after rounding...");
        for_each_tetra([this](const Tuple& t) {
            if (is_inverted(t)) {
                log_and_throw_error("Inverted tet in input after rounding!");
            }
        });
        logger().info("done");
    }
}

void SimWildMesh::init_from_image(
    const MatrixXd& V,
    const MatrixXi& T,
    const MatrixSi& T_tags,
    const std::vector<std::string>& tag_names)
{
    assert(V.cols() == 3);
    assert(T.cols() == 4);
    assert(T_tags.rows() == T.rows());

    init(T);

    assert(check_mesh_connectivity_validity());

    m_tags_count = T_tags.cols();

    m_vertex_attribute.resize(V.rows());
    m_tet_attribute.resize(T.rows());
    m_face_attribute.resize(T.rows() * 4);

    for (int i = 0; i < vert_capacity(); i++) {
        m_vertex_attribute[i].m_posf = V.row(i);
        m_vertex_attribute[i].m_pos = to_rational(m_vertex_attribute[i].m_posf);
        m_vertex_attribute[i].m_is_rounded = true;
    }

    // check for inverted mesh
    {
        bool is_inverted = false;
        for (const Tuple& t : get_tets()) {
            if (is_inverted ^ is_inverted_f(t)) {
                if (!is_inverted) {
                    is_inverted = true;
                } else {
                    log_and_throw_error("Tets with different orientations in the input!");
                }
            }
        }

        if (is_inverted) {
            log_and_throw_error(
                "Input mesh is fully inverted! This should not happen... Might be a bug.");
        }
    }

    // add tags
    for (size_t i = 0; i < (size_t)T_tags.rows(); ++i) {
        for (size_t j = 0; j < m_tags_count; ++j) {
            if (T_tags.coeff(i, j) != 0) {
                m_tet_attribute[i].tags.insert(j);
            }
        }
    }

    // add tag names
    for (size_t i = 0; i < tag_names.size(); ++i) {
        m_tag_id_to_name[i] = tag_names[i];
        m_tag_name_to_id[tag_names[i]] = i;
    }

    init_surfaces_and_boundaries();

    if (m_params.preserve_topology) {
        init_vertex_order();
    }

    // init qualities
    for_each_tetra(
        [this](const Tuple& t) { m_tet_attribute[t.tid(*this)].m_quality = get_quality(t); });
}

void SimWildMesh::init_surfaces_and_boundaries()
{
    const auto faces = get_faces();
    logger().info("F = {}", faces.size());

    // tag surface faces and vertices
    std::vector<Eigen::Vector3i> tempF;
    for (const Tuple& f : faces) {
        SmartTuple ff(*this, f);

        const auto t_opp = ff.switch_tetrahedron();
        if (!t_opp) {
            continue;
        }

        {
            const auto& tag0 = m_tet_attribute[ff.tid()].tags;
            const auto& tag1 = m_tet_attribute[t_opp.value().tid()].tags;
            if (tag0 == tag1) {
                continue;
            }
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

    auto reinit_envelope = [&]() {
        m_envelope = std::make_shared<SampleEnvelope>(!m_sim_params.use_sample_envelope);
        m_envelope->init(m_V_envelope, m_F_envelope, m_envelope_eps);
        logger().info(
            "Envelope: {} (eps {:.6})",
            m_envelope->use_exact ? "EXACT" : "sampled",
            m_envelope_eps);
    };

    if (!m_envelope) {
        logger().info("Init envelope from tet tags");
        // build envelopes
        std::vector<Eigen::Vector3d> tempV(vert_capacity());
        for (int i = 0; i < vert_capacity(); i++) {
            tempV[i] = m_vertex_attribute[i].m_posf;
        }

        m_V_envelope = tempV;
        m_F_envelope = tempF;
        reinit_envelope();
    } else if (m_sim_params.operation == "remeshing") {
        // Deliberately NOT behind check_envelope_at_init, unlike triwild's: the answer is not an
        // assertion but a decision -- if any surface face starts outside, the envelope is rebuilt
        // from the tet tags below. Skipping it would change the envelope the run uses, not just
        // what it reports.
        logger().info("Envelope sanity check");
        bool is_outside = false;
        for_each_face([&](const Tuple& t) {
            const size_t fid = t.fid(*this);
            if (!m_face_attribute.at(fid).m_is_surface_fs) {
                return;
            }
            const auto verts = get_face_vids(t);
            const auto& p0 = m_vertex_attribute[verts[0]].m_posf;
            const auto& p1 = m_vertex_attribute[verts[1]].m_posf;
            const auto& p2 = m_vertex_attribute[verts[2]].m_posf;
            if (m_envelope->is_outside({{p0, p1, p2}})) {
                // logger().warn("Face {} is outside!", verts);
                is_outside = true;
            }
        });
        logger().info("Envelope sanity check done");
        if (!is_outside) {
            logger().info("All surface faces are inside the envelope.");
        } else {
            logger().warn("Some surface faces are outside the envelope. Re-build envelope");
            logger().info("Init envelope from tet tags");
            std::vector<Eigen::Vector3d> tempV(vert_capacity());
            for (int i = 0; i < vert_capacity(); i++) {
                tempV[i] = m_vertex_attribute[i].m_posf;
            }
            m_V_envelope = tempV;
            m_F_envelope = tempF;
            reinit_envelope();
        }
    }

    // track bounding box
    for (size_t i = 0; i < faces.size(); i++) {
        const auto vs = get_face_vertices(faces[i]);
        std::array<size_t, 3> vids = {{vs[0].vid(*this), vs[1].vid(*this), vs[2].vid(*this)}};
        int on_bbox = -1;
        for (int k = 0; k < 3; k++) {
            if (m_vertex_attribute[vids[0]].m_pos[k] == m_sim_params.box_min[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == m_sim_params.box_min[k] &&
                m_vertex_attribute[vids[2]].m_pos[k] == m_sim_params.box_min[k]) {
                on_bbox = k * 2;
                break;
            }
            if (m_vertex_attribute[vids[0]].m_pos[k] == m_sim_params.box_max[k] &&
                m_vertex_attribute[vids[1]].m_pos[k] == m_sim_params.box_max[k] &&
                m_vertex_attribute[vids[2]].m_pos[k] == m_sim_params.box_max[k]) {
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

    if (m_params.preserve_topology) {
        // track open boundaries
        find_order_2_edges();

        int open_boundary_cnt = 0;
        for (const Tuple& e : get_edges()) {
            if (is_order_2_edge(e)) {
                open_boundary_cnt++;
            }
        }
        logger().info("#open boundary edges: {}", open_boundary_cnt);
    }
}

void SimWildMesh::find_order_2_edges()
{
    // for open boundary envelope
    std::vector<Eigen::Vector3d> v_posf(vert_capacity());
    std::vector<Eigen::Vector2i> order_2_edges;

    for (size_t i = 0; i < vert_capacity(); i++) {
        v_posf[i] = m_vertex_attribute[i].m_posf;
    }

    const auto edges = get_edges();
    for (const Tuple& e : edges) {
        const size_t v1 = e.vid(*this);
        const size_t v2 = e.switch_vertex(*this).vid(*this);
        if (is_order_2_edge({{v1, v2}})) {
            order_2_edges.emplace_back(v1, v2);
        }
    }

    logger().info("order 2 edge num: {}", order_2_edges.size());

    if (order_2_edges.size() == 0) {
        return;
    }

    // init open boundary envelope
    //
    // Follows the surface envelope's predicate: same input, same epsilon, so there is no
    // reason for `use_sample_envelope: false` to hold for the surface and not for the order-2
    // curves. It was hard-wired to sampled only because the exact envelope could not be built
    // from edges until fast-envelope #6.
    m_order_2_edge_envelope = std::make_shared<SampleEnvelope>(m_envelope && m_envelope->use_exact);
    // A FRACTION of eps here, deliberately, where the surface envelope uses all of it:
    // widening a boundary-curve envelope buys no unblocking and costs elements. See
    // /order2_envelope_ratio in the spec for the measurement.
    m_order_2_edge_envelope->init(
        v_posf,
        order_2_edges,
        m_params.epsr * m_params.diag_l * m_sim_params.order2_envelope_ratio);
}

bool SimWildMesh::is_order_2_edge(const Tuple& e) const
{
    size_t v1 = e.vid(*this);
    size_t v2 = e.switch_vertex(*this).vid(*this);
    return is_order_2_edge({{v1, v2}});
}

bool SimWildMesh::is_order_2_edge(const std::array<size_t, 2>& e) const
{
    // if (m_vertex_attribute[e[0]].m_order < 2 || m_vertex_attribute[e[1]].m_order < 2) {
    //     return false;
    // }
    return get_order_of_edge(e) == 2;
}

} // namespace wmtk::components::simwild
