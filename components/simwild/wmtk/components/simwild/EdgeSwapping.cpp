#include "SimWildMesh.h"

#include <igl/Timer.h>
#include <wmtk/TetMesh.h>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/ParallelCollect.hpp>
#include <wmtk/utils/RunPass.hpp>
#include "spdlog/spdlog.h"
#include "wmtk/utils/TupleUtils.hpp"

#include <cassert>

namespace wmtk::components::simwild {

void face_attribute_tracker(
    const TetMesh& m,
    const std::vector<size_t>& incident_tets,
    const SimWildMesh::FaceAttCol& m_face_attribute,
    std::map<std::array<size_t, 3>, FaceAttributes>& changed_faces)
{
    changed_faces.clear();
    for (const auto& t : incident_tets) {
        for (int j = 0; j < 4; j++) {
            const TetMesh::Tuple f_t = m.tuple_from_face(t, j);
            auto vids = m.get_face_vids(f_t);
            std::sort(vids.begin(), vids.end());
            changed_faces.try_emplace(vids, m_face_attribute[f_t.fid(m)]);
        }
    }
}

void tracker_assign_after(
    const wmtk::TetMesh& m,
    const std::vector<size_t>& incident_tids,
    const std::map<std::array<size_t, 3>, FaceAttributes>& changed_faces,
    SimWildMesh::FaceAttCol& m_face_attribute)
{
    auto middle_face = std::vector<size_t>();
    auto new_faces = std::set<std::array<size_t, 3>>();

    for (const auto& t : incident_tids) {
        for (auto j = 0; j < 4; j++) {
            auto f_t = m.tuple_from_face(t, j);
            auto global_fid = f_t.fid(m);
            auto vs = m.get_face_vertices(f_t);
            auto vids = std::array<size_t, 3>{{vs[0].vid(m), vs[1].vid(m), vs[2].vid(m)}};
            std::sort(vids.begin(), vids.end());
            auto it = (changed_faces.find(vids));
            if (it == changed_faces.end()) {
                middle_face.push_back(global_fid);
                continue;
            }


            m_face_attribute[global_fid] = it->second; // m_face_attribute[it->second];
        }
    }
    for (const size_t f : middle_face) {
        m_face_attribute[f].reset();
    }
}

void tracker_assign_after(
    const wmtk::TetMesh& m,
    const std::vector<wmtk::TetMesh::Tuple>& incident_tets,
    const std::map<std::array<size_t, 3>, FaceAttributes>& changed_faces,
    SimWildMesh::FaceAttCol& m_face_attribute)
{
    std::vector<size_t> incident_tids;
    incident_tids.reserve(incident_tets.size());
    for (const wmtk::TetMesh::Tuple& t : incident_tets) {
        incident_tids.emplace_back(t.tid(m));
    }

    tracker_assign_after(m, incident_tids, changed_faces, m_face_attribute);
}


size_t SimWildMesh::swap_all_edges_32()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops =
        wmtk::parallel_collect_edge_ops(*this, NUM_THREADS, [](auto&, const auto& e, auto& out) {
            out.emplace_back("edge_swap", e);
        });
    time = timer.getElapsedTime();
    logger().info("edge swap prepare time: {:.4}s", time);
    SurfaceTopoSignature sig_before;
    if (m_sim_params.check_surface_topology) {
        sig_before = surface_topology_signature();
    }
    size_t total_success = 0;
    wmtk::run_pass(
        *this,
        wmtk::PassLock::EdgeTwoRing,
        "edge swap operation",
        [&](auto& executor, auto& mesh) {
            executor.renew_neighbor_tuples = wmtk::renewal_edges;
            executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
            // Retry a failed swap only where the mesh actually changed this round
            // (dirty-epoch localized retry), and report the total across rounds rather
            // than the last round's count -- as tetwild does.
            total_success = wmtk::run_localized_to_convergence(mesh, executor, collect_all_ops);
        });
    if (m_sim_params.check_surface_topology) {
        warn_if_surface_topology_changed(sig_before, "swap_all_edges_32");
    }
    return total_success;
}

bool SimWildMesh::swap_edge_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_before(t)) {
        return false;
    }

    auto incident_tets = get_incident_tids_for_edge(t);
    if (incident_tets.size() != 3) {
        return false;
    }

    auto& cache = swap_cache.local();
    cache.is_surface_flip = false;

    // bbox edges are never swapped.
    if (is_edge_on_bbox(t)) {
        return false;
    }

    const auto& TA = m_tet_attribute;
    // cache.tet_tags = TA[incident_tets[0]].tags;

    // find tag with max count
    {
        std::map<CellTag, size_t> tag_count;
        for (const size_t l : incident_tets) {
            tag_count[TA[l].tags]++;
        }
        if (tag_count.size() > 2) {
            return false; // cannot swap an edge in between 3 different tags
        }
        size_t max_count = 0;
        CellTag max_tag;
        for (const auto& [tag, count] : tag_count) {
            if (count > max_count) {
                max_count = count;
                max_tag = tag;
            }
        }
        cache.tet_tags = max_tag;
    }

    // Surface edges are allowed only as a topology-preserving surface diagonal
    // flip (see prepare_surface_flip_32). If disabled, keep the old behavior of
    // rejecting all surface-edge swaps.
    // Route on the direct incident-surface-face count so a genuine surface edge is never
    // mistaken for interior because of a stale m_is_on_surface flag, which would tear the
    // surface. tetwild routes all three swap paths this way.
    if (edge_incident_surface_face_count(t) > 0) {
        if (!m_sim_params.allow_surface_swap) {
            return false;
        }
        if (!prepare_surface_flip_32(t, incident_tets)) {
            return false;
        }
    }

    double max_energy = -1.0;
    for (const size_t l : incident_tets) {
        max_energy = std::max(TA[l].m_quality, max_energy);
    }
    cache.max_energy = max_energy;

    face_attribute_tracker(
        *this,
        incident_tets,
        m_face_attribute,
        swap_cache.local().changed_faces);

    return true;
}

bool SimWildMesh::prepare_surface_flip_32(const Tuple& t, const std::vector<size_t>& incident_tets)
{
    auto& cache = swap_cache.local();

    const size_t a = t.vid(*this);
    const size_t b = t.switch_vertex(*this).vid(*this);

    // The three "ring" vertices are the incident-tet vertices other than a,b.
    std::array<size_t, 3> ring{};
    int nr = 0;
    for (const size_t tid : incident_tets) {
        const auto vs = oriented_tet_vids(tid);
        for (const size_t v : vs) {
            if (v == a || v == b) {
                continue;
            }
            bool seen = false;
            for (int k = 0; k < nr; ++k) {
                if (ring[k] == v) {
                    seen = true;
                    break;
                }
            }
            if (seen) {
                continue;
            }
            if (nr > 2) {
                log_and_throw_error("prepare_surface_flip_32: more than 3 ring vertices");
                // return false; // not a clean 3->2 ring
            }
            ring[nr++] = v;
        }
    }
    if (nr != 3) {
        log_and_throw_error("prepare_surface_flip_32: not exactly 3 ring vertices");
        // return false;
    }

    // Of the three ring faces (a,b,ring[k]), exactly two must be surface faces
    // (manifold surface edge). Their apexes are the new surface edge (c,d); the
    // remaining apex is the other one (e).
    int n_surf = 0;
    size_t c = 0, d = 0, e = 0; // vertex ids; c/d on surface, e not
    bool e_set = false;
    for (int k = 0; k < 3; ++k) {
        const auto [_, fid] = tuple_from_face(std::array<size_t, 3>{{a, b, ring[k]}});
        (void)_; // tell compiler this variable is not needed
        if (m_face_attribute[fid].m_is_surface_fs) {
            if (n_surf == 0) {
                c = ring[k];
                cache.sf_face_attr = m_face_attribute[fid];
            } else if (n_surf == 1) {
                d = ring[k];
            } else {
                return false; // > 2 surface faces: non-manifold edge
            }
            ++n_surf;
        } else {
            if (e_set) {
                return false; // > 1 non-surface face
            }
            e = ring[k];
            e_set = true;
        }
    }
    if (n_surf != 2 || !e_set) {
        /**
         * This statement should never be reached because the previous loop already checks for the
         * number of surface faces and non-surface faces. If this statement is reached, it indicates
         * a logical error in the code or an unexpected input configuration.
         */
        log_and_throw_error("prepare_surface_flip_32: invalid surface face configuration");
    }

    // The flip adds two surface faces (a,c,d),(b,c,d) on edge (c,d). If any
    // surface face is already incident to edge (c,d), the result is a
    // non-manifold surface edge (> 2 surface faces) -> reject. This counts the
    // incident surface faces directly and does NOT rely on the m_is_on_surface
    // vertex flags (which can be stale), unlike is_edge_on_surface().
    {
        std::array<size_t, 2> cd{{c, d}};
        assert(tuple_from_edge(cd).is_valid(*this));
        const auto sf_edge = get_surface_faces_for_edge(cd);
        if (sf_edge.size() > 0) {
            // (c,d) already has a surface face incident
            return false;
        }
    }

    cache.is_surface_flip = true;
    cache.sf_a = a;
    cache.sf_b = b;
    cache.sf_c = c;
    cache.sf_d = d;
    cache.sf_e = e;
    return true;
}

bool SimWildMesh::swap_edge_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_after(t)) {
        return false;
    }

    // after swap, t points to a face with 2 neighboring tets.
    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");

    const auto& cache = swap_cache.local();

    auto twotets = std::vector<Tuple>{{t, *oppo_tet}};
    auto max_energy = -1.0;
    for (auto& l : twotets) {
        if (is_inverted(l)) {
            return false;
        }
        const double q = get_quality(l);
        m_tet_attribute[l.tid(*this)].m_quality = q;
        max_energy = std::max(q, max_energy);

        m_tet_attribute[l.tid(*this)].tags = cache.tet_tags;
    }
    if (max_energy >= cache.max_energy) {
        return false;
    }

    if (cache.is_surface_flip) {
        // The two new surface faces (a,c,d),(b,c,d) must stay within the
        // Hausdorff envelope, exactly like a surface-edge collapse.
        const auto& VA = m_vertex_attribute;
        if (m_envelope->is_outside(
                {{VA[cache.sf_a].m_posf, VA[cache.sf_c].m_posf, VA[cache.sf_d].m_posf}}))
            return false;
        if (m_envelope->is_outside(
                {{VA[cache.sf_b].m_posf, VA[cache.sf_c].m_posf, VA[cache.sf_d].m_posf}}))
            return false;
    }

    tracker_assign_after(*this, twotets, cache.changed_faces, m_face_attribute);

    if (cache.is_surface_flip) {
        // The generic tracker copied the old (interior) attributes onto the new
        // faces (a,c,d),(b,c,d) and reset the new middle face (c,d,e). Re-tag the
        // two new faces as the flipped surface, carrying the original surface
        // face attributes. Net surface change: -(a,b,c) -(a,b,d) +(a,c,d) +(b,c,d).
        auto [ft1, fid1] =
            tuple_from_face(std::array<size_t, 3>{{cache.sf_a, cache.sf_c, cache.sf_d}});
        auto [ft2, fid2] =
            tuple_from_face(std::array<size_t, 3>{{cache.sf_b, cache.sf_c, cache.sf_d}});
        (void)ft1;
        (void)ft2;
        m_face_attribute[fid1] = cache.sf_face_attr;
        m_face_attribute[fid2] = cache.sf_face_attr;
        /**
         * Setting the m_is_surface_fs flag to true is not necessary. This is already true in
         * cache.sf_face_attr.
         */
        // m_face_attribute[fid1].m_is_surface_fs = true;
        // m_face_attribute[fid2].m_is_surface_fs = true;
        cnt_surface_swap++;
    }

    cnt_swap++;

    return true;
}

size_t SimWildMesh::swap_all_faces()
{
    igl::Timer timer;
    double time;
    timer.start();
    // Faces, not edges. This used to collect edge tuples and queue "face_swap" on them -- 6
    // per tet instead of 4, with each face reachable from up to three of its edges, so the
    // pass enumerated a different set from the one it is named for. tetwild's form.
    auto collect_all_ops =
        wmtk::parallel_collect_face_ops(*this, NUM_THREADS, [](auto&, const auto& f, auto& out) {
            out.emplace_back("face_swap", f);
        });
    time = timer.getElapsedTime();
    logger().info("face swap prepare time: {:.4}s", time);
    size_t total_success = 0;
    wmtk::run_pass(
        *this,
        wmtk::PassLock::FaceTwoRing,
        "face swap operation",
        [&](auto& executor, auto& mesh) {
            executor.renew_neighbor_tuples = wmtk::renewal_faces;
            executor.priority = [](auto& m, auto op, auto& t) { return m.get_length2(t); };
            // Retry a failed swap only where the mesh actually changed this round
            // (dirty-epoch localized retry), and report the total across rounds rather
            // than the last round's count -- as tetwild does.
            total_success = wmtk::run_localized_to_convergence(mesh, executor, collect_all_ops);
        });
    return total_success;
}

bool SimWildMesh::swap_face_before(const Tuple& t)
{
    if (!TetMesh::swap_face_before(t)) {
        return false;
    }
    // if (m_params.preserve_global_topology) return false;

    auto& cache = swap_cache.local();

    const SmartTuple tt(*this, t);

    auto fid = tt.fid();
    if (m_face_attribute[fid].m_is_surface_fs || m_face_attribute[fid].m_is_bbox_fs >= 0) {
        return false;
    }
    auto oppo_tet = tt.switch_tetrahedron();
    assert(oppo_tet.has_value() && "Should not swap boundary.");

    const size_t t0 = tt.tid();
    const size_t t1 = oppo_tet.value().tid();

    const auto& TA = m_tet_attribute;

    const double max_energy = std::max(TA[t0].m_quality, TA[t1].m_quality);

    // pre-compute energy
    {
        const auto t1_vids = oriented_tet_vids(t1);

        const size_t v0 = tt.vid();
        const size_t v1 = tt.switch_vertex().vid();
        const size_t v2 = tt.switch_edge().switch_vertex().vid();
        const size_t v3 = tt.switch_face().switch_edge().switch_vertex().vid();

        std::array<size_t, 3> tri{{v0, v1, v2}};

        for (auto i = 0; i < 3; i++) {
            std::array<size_t, 4> new_tet = t1_vids;
            wmtk::array_replace_inline(new_tet, tri[i], v3);
            if (is_inverted(new_tet)) {
                return false;
            }
            const double q = get_quality(new_tet);
            if (q >= max_energy) {
                return false;
            }
        }
    }

    cache.tet_tags = TA[tt.tid()].tags;
    // if (TA[oppo_tet.value().tid()].tags != cache.tet_tags) {
    //    log_and_throw_error("not all tets have the same tag"); // for debugging
    //}

    std::vector<size_t> twotets{t0, t1};

    face_attribute_tracker(*this, twotets, m_face_attribute, cache.changed_faces);
    return true;
}

bool SimWildMesh::swap_face_after(const Tuple& t)
{
    if (!TetMesh::swap_face_after(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);

    for (auto& l : incident_tets) {
        auto q = get_quality(l);
        m_tet_attribute[l.tid(*this)].m_quality = q;
        m_tet_attribute[l.tid(*this)].tags = swap_cache.local().tet_tags;
    }

    tracker_assign_after(*this, incident_tets, swap_cache.local().changed_faces, m_face_attribute);

    cnt_swap++;
    return true;
}

size_t SimWildMesh::swap_all_edges_all()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops =
        wmtk::parallel_collect_edge_ops(*this, NUM_THREADS, [](auto&, const auto& e, auto& out) {
            out.emplace_back("edge_swap", e);
            out.emplace_back("edge_swap_44", e);
            out.emplace_back("edge_swap_56", e);
        });
    time = timer.getElapsedTime();
    logger().info("edge swap prepare time: {:.4}s", time);
    SurfaceTopoSignature sig_before;
    if (m_sim_params.check_surface_topology) {
        sig_before = surface_topology_signature();
    }
    size_t total_success = 0;
    wmtk::run_pass(
        *this,
        wmtk::PassLock::EdgeTwoRing,
        "edge swap operation",
        [&](auto& executor, auto& mesh) {
            // executor.renew_neighbor_tuples = wmtk::renewal_edges;
            executor.renew_neighbor_tuples =
                [](const TetMesh& m, const std::string& op, const std::vector<Tuple>& newt) {
                    std::vector<std::pair<std::string, TetMesh::Tuple>> op_tups;
                    std::vector<TetMesh::Tuple> new_edges;
                    for (const TetMesh::Tuple& ti : newt) {
                        for (auto j = 0; j < 6; j++) {
                            new_edges.push_back(m.tuple_from_edge(ti.tid(m), j));
                        }
                    };
                    wmtk::unique_edge_tuples(m, new_edges);
                    op_tups.reserve(new_edges.size() * 3);
                    for (const Tuple& loc : new_edges) {
                        op_tups.emplace_back("edge_swap", loc);
                        op_tups.emplace_back("edge_swap_44", loc);
                        op_tups.emplace_back("edge_swap_56", loc);
                    }
                    return op_tups;
                };
            executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
            // Retry a failed swap only where the mesh actually changed this round
            // (dirty-epoch localized retry), and report the total across rounds rather
            // than the last round's count -- as tetwild does.
            total_success = wmtk::run_localized_to_convergence(mesh, executor, collect_all_ops);
        });
    if (m_sim_params.check_surface_topology) {
        warn_if_surface_topology_changed(sig_before, "swap_all_edges_32");
    }
    return total_success;
}


size_t SimWildMesh::swap_all_edges_44()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops =
        wmtk::parallel_collect_edge_ops(*this, NUM_THREADS, [](auto&, const auto& e, auto& out) {
            out.emplace_back("edge_swap_44", e);
        });
    time = timer.getElapsedTime();
    logger().info("edge swap 44 prepare time: {:.4}s", time);
    size_t total_success = 0;
    wmtk::run_pass(
        *this,
        wmtk::PassLock::EdgeTwoRing,
        "edge swap 44 operation",
        [&](auto& executor, auto& mesh) {
            executor.renew_neighbor_tuples = wmtk::renewal_edges;
            executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
            // Retry a failed swap only where the mesh actually changed this round
            // (dirty-epoch localized retry), and report the total across rounds rather
            // than the last round's count -- as tetwild does.
            total_success = wmtk::run_localized_to_convergence(mesh, executor, collect_all_ops);
        });
    return total_success;
}

bool SimWildMesh::swap_edge_44_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_44_before(t)) {
        return false;
    }
    // if (m_params.preserve_global_topology) return false;

    auto incident_tets = get_incident_tids_for_edge(t);
    if (incident_tets.size() != 4) {
        return false;
    }

    if (edge_incident_surface_face_count(t) > 0 || is_edge_on_bbox(t)) {
        return false;
    }

    auto& cache = swap_cache.local();
    const auto& TA = m_tet_attribute;

    cache.tet_tags = TA[incident_tets[0]].tags;
    double max_energy = -1.0;
    for (const size_t l : incident_tets) {
        max_energy = std::max(TA[l].m_quality, max_energy);
        // if (TA[l].tags != cache.tet_tags) {
        //     log_and_throw_error(
        //         "not all tets have the same tags. {} != {}",
        //         cache.tet_tags,
        //        TA[l].tags); // for debugging
        //}
    }
    cache.max_energy = max_energy;

    face_attribute_tracker(*this, incident_tets, m_face_attribute, cache.changed_faces);

    return true;
}

bool SimWildMesh::swap_edge_44_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_44_after(t)) return false;

    const auto incident_tets = get_incident_tets_for_edge(t);

    double max_energy = -1.0;
    for (auto& l : incident_tets) {
        if (is_inverted(l)) return false;
        auto q = get_quality(l);
        m_tet_attribute[l.tid(*this)].m_quality = q;
        max_energy = std::max(q, max_energy);

        m_tet_attribute[l.tid(*this)].tags = swap_cache.local().tet_tags;
    }

    if (max_energy >= swap_cache.local().max_energy) {
        return false;
    }

    tracker_assign_after(*this, incident_tets, swap_cache.local().changed_faces, m_face_attribute);

    cnt_swap++;
    return true;
}

size_t SimWildMesh::swap_all_edges_56()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops =
        wmtk::parallel_collect_edge_ops(*this, NUM_THREADS, [](auto&, const auto& e, auto& out) {
            out.emplace_back("edge_swap_56", e);
        });
    time = timer.getElapsedTime();
    logger().info("edge swap 56 prepare time: {:.4}s", time);
    size_t total_success = 0;
    wmtk::run_pass(
        *this,
        wmtk::PassLock::EdgeTwoRing,
        "edge swap 56 operation",
        [&](auto& executor, auto& mesh) {
            executor.renew_neighbor_tuples = wmtk::renewal_edges;
            executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
            // Retry a failed swap only where the mesh actually changed this round
            // (dirty-epoch localized retry), and report the total across rounds rather
            // than the last round's count -- as tetwild does.
            total_success = wmtk::run_localized_to_convergence(mesh, executor, collect_all_ops);
        });
    return total_success;
}

bool SimWildMesh::swap_edge_56_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_56_before(t)) {
        return false;
    }

    const auto incident_tets = get_incident_tids_for_edge(t);
    if (incident_tets.size() != 5) {
        return false;
    }
    if (edge_incident_surface_face_count(t) > 0 || is_edge_on_bbox(t)) {
        return false;
    }

    auto& cache = swap_cache.local();
    const auto& TA = m_tet_attribute;

    cache.tet_tags = TA[incident_tets[0]].tags;
    double max_energy = -1.0;
    for (const size_t l : incident_tets) {
        max_energy = std::max(TA[l].m_quality, max_energy);
        // if (TA[l].tags != cache.tet_tags) {
        //     log_and_throw_error(
        //         "not all tets have the same tags. {} != {}",
        //         cache.tet_tags,
        //        TA[l].tags); // for debugging
        //}
    }

    swap_cache.local().max_energy = max_energy;

    face_attribute_tracker(
        *this,
        incident_tets,
        m_face_attribute,
        swap_cache.local().changed_faces);

    return true;
}

bool SimWildMesh::swap_edge_56_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_56_after(t)) {
        return false;
    }

    /**
     * There is no need to check for inversion or energy here. The operation would have been
     * rejected already due to `swap_edge_56_energy()`.
     */

    const auto e1 = get_incident_tids_for_edge(t);
    const auto e2 = get_incident_tids_for_edge(t.switch_edge(*this));
    const auto tids = wmtk::set_union(e1, e2);

    double max_energy = -1.0;
    for (const size_t tid : tids) {
        const Tuple tet = tuple_from_tet(tid);
        if (is_inverted(tet)) return false;
        auto q = get_quality(tet);
        m_tet_attribute[tid].m_quality = q;
        max_energy = std::max(q, max_energy);

        m_tet_attribute[tid].tags = swap_cache.local().tet_tags;
    }

    tracker_assign_after(*this, tids, swap_cache.local().changed_faces, m_face_attribute);

    cnt_swap++;
    return true;
}

} // namespace wmtk::components::simwild