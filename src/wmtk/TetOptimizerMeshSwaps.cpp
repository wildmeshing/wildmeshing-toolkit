#include <wmtk/TetMesh.h>
#include <wmtk/TetOptimizerMesh.h>

#include <igl/Timer.h>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/ParallelCollect.hpp>
#include <wmtk/utils/RunPass.hpp>
#include "spdlog/spdlog.h"
#include "wmtk/utils/TupleUtils.hpp"

#include <cassert>
#include <limits>

namespace wmtk {

using FaceAttributes = TetOptimizerMesh::FaceAttributes;

void face_attribute_tracker(
    const TetMesh& m,
    const std::vector<size_t>& incident_tets,
    const TetOptimizerMesh::FaceAttCol& m_face_attribute,
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
    TetOptimizerMesh::FaceAttCol& m_face_attribute)
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
    TetOptimizerMesh::FaceAttCol& m_face_attribute)
{
    std::vector<size_t> incident_tids;
    incident_tids.reserve(incident_tets.size());
    for (const wmtk::TetMesh::Tuple& t : incident_tets) {
        incident_tids.emplace_back(t.tid(m));
    }

    tracker_assign_after(m, incident_tids, changed_faces, m_face_attribute);
}


size_t TetOptimizerMesh::swap_all_edges_32()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops = wmtk::parallel_collect_edge_ops(
        *this,
        NUM_THREADS,
        [](TetOptimizerMesh&, const Tuple& e, auto& out) { out.emplace_back("edge_swap", e); });
    time = timer.getElapsedTime();
    logger().info("edge swap prepare time: {:.4}s", time);
    size_t total_success = 0;
    SurfaceTopoSignature sig_before;
    if (check_surface_topology()) {
        sig_before = surface_topology_signature();
    }
    wmtk::run_pass(
        *this,
        wmtk::PassLock::EdgeTwoRing,
        "edge swap operation",
        [&](auto& executor, auto& mesh) {
            executor.renew_neighbor_tuples = wmtk::renewal_edges;
            executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
            total_success = wmtk::run_localized_to_convergence(mesh, executor, collect_all_ops);
        });
    if (check_surface_topology()) {
        warn_if_surface_topology_changed(sig_before, "swap_all_edges_32");
    }
    return total_success;
}

bool TetOptimizerMesh::swap_edge_before(const Tuple& t)
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
    // Surface edges are allowed only as a topology-preserving surface diagonal flip (see
    // prepare_surface_flip). If disabled, keep the old behavior of rejecting all surface-edge
    // swaps. Route on the direct incident-surface-face count so a genuine surface edge is never
    // mistaken for interior because of stale m_is_on_surface flags (which would tear the surface).
    const int n_surf_faces = edge_incident_surface_face_count(t);
    if (n_surf_faces > 0) {
        if (!allow_surface_swap()) return false;
        if (!prepare_surface_flip(t, incident_tets)) return false;
    } else if (!swap_before_interior(incident_tets)) {
        return false;
    }

    double max_energy = -1.0;
    for (const size_t l : incident_tets) {
        max_energy = std::max(cell_quality(l), max_energy);
    }
    cache.max_energy = max_energy;

    face_attribute_tracker(*this, incident_tets, m_face_attribute, cache.changed_faces);

    return true;
}

bool TetOptimizerMesh::prepare_surface_flip(
    const Tuple& t,
    const std::vector<size_t>& incident_tets)
{
    auto& cache = swap_cache.local();
    cache.is_surface_flip = false;

    const size_t a = t.vid(*this);
    const size_t b = t.switch_vertex(*this).vid(*this);

    // Flipping an open-boundary edge would change the surface's boundary loops.
    if (is_open_boundary_edge(t)) {
        return false;
    }

    // The "ring" vertices are the incident-tet vertices other than a,b (deduplicated). This works
    // for any ring size (3->2, 4-4, 5-6); the specific retetrahedralization that realizes the flip
    // is selected later by swap_edge_44_accept_case / swap_edge_56_accept_case.
    std::vector<size_t> ring;
    ring.reserve(incident_tets.size() + 1);
    for (const size_t tid : incident_tets) {
        for (const size_t v : oriented_tet_vids(tid)) {
            if (v == a || v == b) continue;
            if (std::find(ring.begin(), ring.end(), v) == ring.end()) ring.push_back(v);
        }
    }

    // Exactly two of the ring faces (a,b,r) must be surface faces (manifold surface edge). Their
    // apexes are the endpoints (c,d) of the new surface edge.
    int n_surf = 0;
    size_t c = 0, d = 0;
    for (const size_t r : ring) {
        auto [ftup, fid] = tuple_from_face(std::array<size_t, 3>{{a, b, r}});
        (void)ftup;
        if (fid == static_cast<size_t>(-1)) continue;
        if (!m_face_attribute[fid].m_is_surface_fs) continue;
        if (n_surf == 0) {
            c = r;
            cache.sf_face_attr = m_face_attribute[fid];
        } else if (n_surf == 1) {
            d = r;
        } else {
            return false; // > 2 surface faces: non-manifold edge
        }
        ++n_surf;
    }
    if (n_surf != 2) return false;

    // The flip adds two surface faces (a,c,d),(b,c,d) on edge (c,d). If any surface face is
    // already incident to edge (c,d), the result is a non-manifold surface edge (> 2 surface
    // faces) -> reject. This counts the incident surface faces directly and does NOT rely on the
    // m_is_on_surface vertex flags (which can be stale), unlike is_edge_on_surface().
    //
    // (c,d) need not exist yet: for a 3->2 flip the two apexes are adjacent, but for 4-4 / 5-6
    // they are not, so the edge is created by the flip. Hence the validity check rather than an
    // assert -- asserting here is a 3->2-only invariant and aborts a Debug build on 4-4.
    {
        const Tuple cd = tuple_from_edge(std::array<size_t, 2>{{c, d}});
        if (cd.is_valid(*this)) {
            const auto cd_tets = get_incident_tets_for_edge(cd);
            for (const auto& ct : cd_tets) {
                const auto vs = oriented_tet_vids(ct.tid(*this));
                for (const size_t w : vs) {
                    if (w == c || w == d) continue;
                    auto [wf, wfid] = tuple_from_face(std::array<size_t, 3>{{c, d, w}});
                    (void)wf;
                    if (wfid != static_cast<size_t>(-1) && m_face_attribute[wfid].m_is_surface_fs)
                        return false;
                }
            }
        }
    }

    // The two would-be new surface faces (a,c,d),(b,c,d) must not already be tagged surface,
    // otherwise the flip corrupts manifoldness / boundaries.
    {
        // These are the faces the flip is about to create, so on the normal path they do not
        // exist yet. tuple_from_face treats that as a programming error -- it asserts, and only
        // returns the -1 sentinel once asserts are compiled out -- so ask whether the three
        // vertices share a tet before querying, rather than letting a Debug build abort on
        // every 4-4 flip.
        const auto face_is_surface = [&](size_t x, size_t y, size_t z) {
            if (lowest_common_tet(x, y, z) == std::numeric_limits<size_t>::max()) {
                return false; // no such face yet, so it cannot already be tagged surface
            }
            const auto [ftup, fid] = tuple_from_face(std::array<size_t, 3>{{x, y, z}});
            (void)ftup;
            return fid != static_cast<size_t>(-1) && m_face_attribute[fid].m_is_surface_fs;
        };
        if (face_is_surface(a, c, d)) return false;
        if (face_is_surface(b, c, d)) return false;
    }

    if (!swap_before_surface(incident_tets, a, b, c, d)) {
        return false;
    }

    cache.is_surface_flip = true;
    cache.sf_a = a;
    cache.sf_b = b;
    cache.sf_c = c;
    cache.sf_d = d;
    return true;
}

bool TetOptimizerMesh::swap_edge_44_accept_case(const std::array<size_t, 2>& new_edge)
{
    const auto& cache = swap_cache.local();
    if (!cache.is_surface_flip) return true; // interior edge: keep pure min-energy behavior
    // Surface flip: accept only the diagonal that creates the new surface edge (c,d).
    return (new_edge[0] == cache.sf_c && new_edge[1] == cache.sf_d) ||
           (new_edge[0] == cache.sf_d && new_edge[1] == cache.sf_c);
}

bool TetOptimizerMesh::swap_edge_56_accept_case(const std::array<size_t, 3>& new_face)
{
    const auto& cache = swap_cache.local();
    if (!cache.is_surface_flip) return true; // interior edge: keep pure min-energy behavior
    // Surface flip: the fan apex (new_face[0]) must be one surface apex and the other surface apex
    // must be one of the two fan tips, so the created diagonal is exactly (c,d). Keying on the apex
    // (not "the face contains edge (c,d)") rejects the spurious case where (c,d) is a pre-existing
    // link edge of the fan face rather than a newly created diagonal.
    const size_t apex = new_face[0];
    size_t other;
    if (apex == cache.sf_c)
        other = cache.sf_d;
    else if (apex == cache.sf_d)
        other = cache.sf_c;
    else
        return false;
    return new_face[1] == other || new_face[2] == other;
}

bool TetOptimizerMesh::swap_edge_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_after(t)) {
        return false;
    }

    const auto& cache = swap_cache.local();

    // after swap, t points to a face with 2 neighboring tets.
    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");

    std::vector<Tuple> twotets{{t, *oppo_tet}};
    double max_energy = -1.0;
    for (const Tuple& l : twotets) {
        if (is_inverted(l)) {
            return false;
        }
        const double q = get_quality(l);
        set_cell_quality(l.tid(*this), q);
        max_energy = std::max(q, max_energy);
    }
    if (max_energy >= cache.max_energy) {
        return false;
    }

    std::vector<size_t> new_tids;
    new_tids.reserve(twotets.size());
    for (const Tuple& tet : twotets) new_tids.push_back(tet.tid(*this));
    if (!swap_after_cells(new_tids, cache.is_surface_flip)) {
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
        cnt_surface_swap_32++;
    }

    cnt_swap++;

    return true;
}


size_t TetOptimizerMesh::swap_all_faces()
{
    igl::Timer timer;
    double time;
    timer.start();
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
            total_success = wmtk::run_localized_to_convergence(mesh, executor, collect_all_ops);
        });
    return total_success;
}

bool TetOptimizerMesh::swap_face_before(const Tuple& t)
{
    if (!TetMesh::swap_face_before(t)) {
        return false;
    }
    // if (m_params.preserve_global_topology) return false;

    auto& cache = swap_cache.local();
    cache.is_surface_flip = false;

    const SmartTuple tt(*this, t);

    auto fid = tt.fid();
    if (m_face_attribute[fid].m_is_surface_fs || m_face_attribute[fid].m_is_bbox_fs >= 0) {
        return false;
    }
    auto oppo_tet = tt.switch_tetrahedron();
    assert(oppo_tet.has_value() && "Should not swap boundary.");

    const size_t t0 = tt.tid();
    const size_t t1 = oppo_tet.value().tid();

    const double max_energy = std::max(cell_quality(t0), cell_quality(t1));

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

    std::vector<size_t> twotets{t0, t1};

    if (!swap_before_interior(twotets)) {
        return false;
    }

    face_attribute_tracker(*this, twotets, m_face_attribute, cache.changed_faces);
    return true;
}

bool TetOptimizerMesh::swap_face_after(const Tuple& t)
{
    if (!TetMesh::swap_face_after(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);

    for (auto& l : incident_tets) {
        auto q = get_quality(l);
        set_cell_quality(l.tid(*this), q);
    }

    std::vector<size_t> new_tids;
    new_tids.reserve(incident_tets.size());
    for (const Tuple& tet : incident_tets) new_tids.push_back(tet.tid(*this));
    if (!swap_after_cells(new_tids, false)) return false;

    tracker_assign_after(*this, incident_tets, swap_cache.local().changed_faces, m_face_attribute);

    cnt_swap++;
    return true;
}

size_t TetOptimizerMesh::swap_all_edges_all()
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
    size_t total_success = 0;
    SurfaceTopoSignature sig_before;
    if (check_surface_topology()) {
        sig_before = surface_topology_signature();
    }
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
            total_success = wmtk::run_localized_to_convergence(mesh, executor, collect_all_ops);
        });
    if (check_surface_topology()) {
        warn_if_surface_topology_changed(sig_before, "swap_all_edges_all");
    }
    return total_success;
}


size_t TetOptimizerMesh::swap_all_edges_44()
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
    SurfaceTopoSignature sig_before;
    if (check_surface_topology()) sig_before = surface_topology_signature();
    wmtk::run_pass(
        *this,
        wmtk::PassLock::EdgeTwoRing,
        "edge swap 44 operation",
        [&](auto& executor, auto& mesh) {
            executor.renew_neighbor_tuples = wmtk::renewal_edges;
            executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
            total_success = wmtk::run_localized_to_convergence(mesh, executor, collect_all_ops);
        });
    if (check_surface_topology()) warn_if_surface_topology_changed(sig_before, "swap_all_edges_44");
    return total_success;
}

bool TetOptimizerMesh::swap_edge_44_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_44_before(t)) {
        return false;
    }

    auto& cache = swap_cache.local();
    cache.is_surface_flip = false;

    auto incident_tets = get_incident_tids_for_edge(t);
    if (incident_tets.size() != 4) {
        return false;
    }

    // bbox edges are never swapped.
    if (is_edge_on_bbox(t)) {
        return false;
    }
    // Surface edges are allowed only as a topology-preserving surface diagonal flip. The base 4-4
    // swap is steered to the case that creates the new surface edge (c,d) by
    // swap_edge_44_accept_case; if no 4-4 diagonal yields (c,d) the swap is rejected. Route on the
    // direct incident-surface-face count so a genuine surface edge is never mistaken for interior.
    const int n_surf_faces = edge_incident_surface_face_count(t);
    if (n_surf_faces > 0) {
        if (!allow_surface_swap()) return false;
        if (!prepare_surface_flip(t, incident_tets)) return false;
    } else if (!swap_before_interior(incident_tets)) {
        return false;
    }

    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        max_energy = std::max(cell_quality(l), max_energy);
    }
    cache.max_energy = max_energy;

    face_attribute_tracker(*this, incident_tets, m_face_attribute, cache.changed_faces);

    return true;
}

bool TetOptimizerMesh::swap_edge_44_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_44_after(t)) return false;

    auto incident_tets = get_incident_tets_for_edge(t);

    auto& cache = swap_cache.local();
    auto max_energy = -1.0;
    for (auto& l : incident_tets) {
        if (is_inverted(l)) return false;
        auto q = get_quality(l);
        set_cell_quality(l.tid(*this), q);
        max_energy = std::max(q, max_energy);
    }

    if (max_energy >= cache.max_energy) {
        return false;
    }

    std::vector<size_t> new_tids;
    new_tids.reserve(incident_tets.size());
    for (const Tuple& tet : incident_tets) new_tids.push_back(tet.tid(*this));
    if (!swap_after_cells(new_tids, cache.is_surface_flip)) return false;

    if (cache.is_surface_flip) {
        // The two new surface faces (a,c,d),(b,c,d) must stay within the Hausdorff envelope,
        // exactly like a surface-edge collapse / the 3->2 surface flip.
        const auto& VA = m_vertex_attribute;
        if (m_envelope->is_outside(
                {{VA[cache.sf_a].m_posf, VA[cache.sf_c].m_posf, VA[cache.sf_d].m_posf}}))
            return false;
        if (m_envelope->is_outside(
                {{VA[cache.sf_b].m_posf, VA[cache.sf_c].m_posf, VA[cache.sf_d].m_posf}}))
            return false;
    }

    tracker_assign_after(*this, incident_tets, cache.changed_faces, m_face_attribute);

    if (cache.is_surface_flip) {
        // Re-tag the two new surface faces (the generic tracker reset them to interior). Net
        // surface change: -(a,b,c) -(a,b,d) +(a,c,d) +(b,c,d).
        auto [ft1, fid1] =
            tuple_from_face(std::array<size_t, 3>{{cache.sf_a, cache.sf_c, cache.sf_d}});
        auto [ft2, fid2] =
            tuple_from_face(std::array<size_t, 3>{{cache.sf_b, cache.sf_c, cache.sf_d}});
        (void)ft1;
        (void)ft2;
        m_face_attribute[fid1] = cache.sf_face_attr;
        m_face_attribute[fid2] = cache.sf_face_attr;
        m_face_attribute[fid1].m_is_surface_fs = true;
        m_face_attribute[fid2].m_is_surface_fs = true;
        cnt_surface_swap++;
        cnt_surface_swap_44++;
    }

    cnt_swap++;
    return true;
}

size_t TetOptimizerMesh::swap_all_edges_56()
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
    SurfaceTopoSignature sig_before;
    if (check_surface_topology()) sig_before = surface_topology_signature();
    wmtk::run_pass(
        *this,
        wmtk::PassLock::EdgeTwoRing,
        "edge swap 56 operation",
        [&](auto& executor, auto& mesh) {
            executor.renew_neighbor_tuples = wmtk::renewal_edges;
            executor.priority = [&](auto& m, auto op, auto& t) { return m.get_length2(t); };
            total_success = wmtk::run_localized_to_convergence(mesh, executor, collect_all_ops);
        });
    if (check_surface_topology()) warn_if_surface_topology_changed(sig_before, "swap_all_edges_56");
    return total_success;
}

bool TetOptimizerMesh::swap_edge_56_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_56_before(t)) {
        return false;
    }

    auto& cache = swap_cache.local();
    cache.is_surface_flip = false;

    const auto incident_tets = get_incident_tids_for_edge(t);
    if (incident_tets.size() != 5) {
        return false;
    }
    // bbox edges are never swapped.
    if (is_edge_on_bbox(t)) {
        return false;
    }
    // Surface edges are allowed only as a topology-preserving surface diagonal flip. The base 5-6
    // swap is steered to the fan that creates the new surface edge (c,d) by
    // swap_edge_56_accept_case; if no fan yields (c,d) the swap is rejected. Route on the direct
    // incident-surface-face count so a genuine surface edge is never mistaken for interior.
    const int n_surf_faces = edge_incident_surface_face_count(t);
    if (n_surf_faces > 0) {
        if (!allow_surface_swap()) return false;
        if (!prepare_surface_flip(t, incident_tets)) return false;
    } else if (!swap_before_interior(incident_tets)) {
        return false;
    }

    double max_energy = -1.0;
    for (const size_t l : incident_tets) {
        max_energy = std::max(cell_quality(l), max_energy);
    }
    cache.max_energy = max_energy;

    face_attribute_tracker(*this, incident_tets, m_face_attribute, cache.changed_faces);

    return true;
}

bool TetOptimizerMesh::swap_edge_56_after(const Tuple& t)
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

    auto& cache = swap_cache.local();
    double max_energy = -1.0;
    for (const size_t tid : tids) {
        const Tuple tet = tuple_from_tet(tid);
        auto q = get_quality(tet);
        set_cell_quality(tid, q);
        max_energy = std::max(q, max_energy);
    }

    if (!swap_after_cells(tids, cache.is_surface_flip)) return false;

    if (cache.is_surface_flip) {
        // The two new surface faces (a,c,d),(b,c,d) must stay within the Hausdorff envelope.
        const auto& VA = m_vertex_attribute;
        if (m_envelope->is_outside(
                {{VA[cache.sf_a].m_posf, VA[cache.sf_c].m_posf, VA[cache.sf_d].m_posf}}))
            return false;
        if (m_envelope->is_outside(
                {{VA[cache.sf_b].m_posf, VA[cache.sf_c].m_posf, VA[cache.sf_d].m_posf}}))
            return false;
    }

    tracker_assign_after(*this, tids, cache.changed_faces, m_face_attribute);

    if (cache.is_surface_flip) {
        // Re-tag the two new surface faces (the generic tracker reset them to interior). Net
        // surface change: -(a,b,c) -(a,b,d) +(a,c,d) +(b,c,d).
        auto [ft1, fid1] =
            tuple_from_face(std::array<size_t, 3>{{cache.sf_a, cache.sf_c, cache.sf_d}});
        auto [ft2, fid2] =
            tuple_from_face(std::array<size_t, 3>{{cache.sf_b, cache.sf_c, cache.sf_d}});
        (void)ft1;
        (void)ft2;
        m_face_attribute[fid1] = cache.sf_face_attr;
        m_face_attribute[fid2] = cache.sf_face_attr;
        m_face_attribute[fid1].m_is_surface_fs = true;
        m_face_attribute[fid2].m_is_surface_fs = true;
        cnt_surface_swap++;
        cnt_surface_swap_56++;
    }

    cnt_swap++;
    return true;
}

} // namespace wmtk
