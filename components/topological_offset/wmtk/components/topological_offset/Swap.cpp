
#include "TopoOffsetTetMesh.h"

#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/ParallelCollect.hpp>

namespace wmtk::components::topological_offset {

bool TopoOffsetTetMesh::swap_edge_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_before(t)) {
        return false;
    }

    const size_t a = t.vid(*this);
    const size_t b = t.switch_vertex(*this).vid(*this);

    if (m_vertex_attribute[a].m_is_on_surface || m_vertex_attribute[b].m_is_on_surface) {
        return false; // never touch the input surface
    }
    if (!get_offset_surface_faces_for_vertex(t).empty() ||
        !get_offset_surface_faces_for_vertex(t.switch_vertex(*this)).empty()) {
        return false; // never touch the offset surface
    }

    const std::vector<size_t> incident_tids = get_incident_tids_for_edge(t);
    if (incident_tids.size() != 3) {
        return false; // swap_edge (3-2) is only defined for edges with exactly 3 incident tets
    }

    auto& cache = swap_edge_cache.local();

    // Confine the swap to a single homogeneous-label neighborhood. This is what lets the new
    // tets/faces below unambiguously inherit a label/tag instead of having to guess one, and
    // it is also what keeps a swap from ever moving the offset/input region boundary: since
    // that boundary lives entirely in *which* tets carry which label, a swap that never mixes
    // labels cannot touch it, regardless of local geometry.
    cache.common_label = m_tet_attribute[incident_tids[0]].label;
    cache.common_tag = m_tet_attribute[incident_tids[0]].tag;
    cache.max_quality_before = 0.;
    for (const size_t tid : incident_tids) {
        if (m_tet_attribute[tid].label != cache.common_label ||
            m_tet_attribute[tid].tag != cache.common_tag) {
            return false;
        }
        cache.max_quality_before = std::max(cache.max_quality_before, m_tet_attribute[tid].m_quality);
    }

    // Snapshot every edge/face of the 3 incident tets, keyed by vertex ids for the same reason
    // as collapse: eid()/fid() can point at a different (unwritten) slot once the tets that
    // touched (a, b) are replaced. Excluded here are exactly the simplices that vanish with
    // the swap: edge (a, b) itself, and its 3 incident faces (each contains both a and b).
    cache.edge_labels.clear();
    cache.face_labels.clear();
    for (const size_t tid : incident_tids) {
        for (int i = 0; i < 6; ++i) {
            const Tuple e = tuple_from_edge(tid, i);
            const size_t ev0 = e.vid(*this);
            const size_t ev1 = e.switch_vertex(*this).vid(*this);
            if ((ev0 == a && ev1 == b) || (ev0 == b && ev1 == a)) continue; // vanishes
            cache.edge_labels[simplex::Edge(ev0, ev1)] = m_edge_attribute[e.eid(*this)].label;
        }
        for (int i = 0; i < 4; ++i) {
            const Tuple f = tuple_from_face(tid, i);
            const std::array<Tuple, 3> fv = get_face_vertices(f);
            const size_t fv0 = fv[0].vid(*this);
            const size_t fv1 = fv[1].vid(*this);
            const size_t fv2 = fv[2].vid(*this);
            const bool has_a = (fv0 == a || fv1 == a || fv2 == a);
            const bool has_b = (fv0 == b || fv1 == b || fv2 == b);
            if (has_a && has_b) continue; // vanishes
            cache.face_labels[simplex::Face(fv0, fv1, fv2)] = m_face_attribute[f.fid(*this)].label;
        }
    }

    return true;
}

bool TopoOffsetTetMesh::swap_edge_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_after(t)) {
        return false;
    }

    auto& cache = swap_edge_cache.local();

    // After a 3-2 swap, t is anchored on the newly created face, shared by the 2 new tets.
    const auto oppo = t.switch_tetrahedron(*this);
    assert(oppo.has_value());
    const std::array<size_t, 2> new_tids{{t.tid(*this), oppo->tid(*this)}};

    double max_quality_after = 0.;
    for (const size_t tid : new_tids) {
        const double q = get_quality(tuple_from_tet(tid));
        m_tet_attribute[tid].m_quality = q;
        m_tet_attribute[tid].label = cache.common_label;
        m_tet_attribute[tid].tag = cache.common_tag;
        max_quality_after = std::max(max_quality_after, q);
    }
    // Inversion is caught by invariants(), called right after this by TetMesh::swap_edge.
    // Only accept swaps that strictly improve the worst quality among the affected tets.
    if (max_quality_after >= cache.max_quality_before) {
        return false;
    }

    // Restore every edge/face that survives the swap by vertex-id lookup. The one simplex
    // that cannot be found -- the newly flipped-in triangle shared by the 2 new tets -- was
    // never cached (it did not exist before the swap), so it falls back to the region's
    // common label instead.
    for (const size_t tid : new_tids) {
        for (int i = 0; i < 6; ++i) {
            const Tuple e = tuple_from_edge(tid, i);
            const size_t ev0 = e.vid(*this);
            const size_t ev1 = e.switch_vertex(*this).vid(*this);
            const auto it = cache.edge_labels.find(simplex::Edge(ev0, ev1));
            m_edge_attribute[e.eid(*this)].label =
                (it != cache.edge_labels.end()) ? it->second : cache.common_label;
        }
        for (int i = 0; i < 4; ++i) {
            const Tuple f = tuple_from_face(tid, i);
            const std::array<Tuple, 3> fv = get_face_vertices(f);
            const auto key =
                simplex::Face(fv[0].vid(*this), fv[1].vid(*this), fv[2].vid(*this));
            const auto it = cache.face_labels.find(key);
            m_face_attribute[f.fid(*this)].label =
                (it != cache.face_labels.end()) ? it->second : cache.common_label;
        }
    }

    return true;
}

void TopoOffsetTetMesh::swap_all_edges()
{
    // 2-3/3-2 flip only -- see the class-level note on swap_all_edges() for why 4-4, 5-6, and
    // face swap are left out of this first draft.
    std::vector<std::pair<std::string, Tuple>> all_ops = wmtk::parallel_collect_edge_ops(
        *this,
        NUM_THREADS,
        [](TopoOffsetTetMesh&, const Tuple& e, auto& out) { out.emplace_back("edge_swap", e); });

    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = [](const auto& m, auto op, const auto& newts) {
            std::vector<std::pair<std::string, Tuple>> op_tups;
            for (const Tuple& t : newts) {
                for (int i = 0; i < 6; ++i) {
                    op_tups.emplace_back(op, m.tuple_from_edge(t.tid(m), i));
                }
            }
            return op_tups;
        };
        // longest edges first, matching SimWildMesh's swap priority: swapping is primarily a
        // quality fix for long/skinny tets, unlike collapse's shortest-first order
        executor.priority = [](const TopoOffsetTetMesh& m, wmtk::Op, const Tuple& t) {
            const size_t v0 = t.vid(m);
            const size_t v1 = t.switch_vertex(m).vid(m);
            return (m.m_vertex_attribute[v0].m_posf - m.m_vertex_attribute[v1].m_posf)
                .squaredNorm();
        };
        wmtk::run_localized_to_convergence(*this, executor, all_ops);
    };

    if (NUM_THREADS > 0) {
        compute_vertex_partition();
        auto executor = wmtk::ExecutePass<TopoOffsetTetMesh>(wmtk::ExecutionPolicy::kPartition);
        executor.lock_vertices = [](TopoOffsetTetMesh& m, const Tuple& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        executor.num_threads = NUM_THREADS;
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<TopoOffsetTetMesh>(wmtk::ExecutionPolicy::kSeq);
        setup_and_execute(executor);
    }
}

} // namespace wmtk::components::topological_offset
