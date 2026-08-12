
#include "TopoOffsetTetMesh.h"

#include <wmtk/utils/VectorUtils.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/ParallelCollect.hpp>

#include <algorithm>
#include <cmath>
#include <limits>

namespace wmtk::components::topological_offset {

void face_attribute_tracker(
    const TetMesh& m,
    const std::vector<size_t>& incident_tets,
    const TopoOffsetTetMesh::FaceAttCol& m_face_attribute,
    std::map<std::array<size_t, 3>, TopoOffsetTetMesh::FaceAttributes>& changed_faces)
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
    const std::map<std::array<size_t, 3>, TopoOffsetTetMesh::FaceAttributes>& changed_faces,
    TopoOffsetTetMesh::FaceAttCol& m_face_attribute)
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
    const std::map<std::array<size_t, 3>, TopoOffsetTetMesh::FaceAttributes>& changed_faces,
    TopoOffsetTetMesh::FaceAttCol& m_face_attribute)
{
    std::vector<size_t> incident_tids;
    incident_tids.reserve(incident_tets.size());
    for (const wmtk::TetMesh::Tuple& t : incident_tets) {
        incident_tids.emplace_back(t.tid(m));
    }

    tracker_assign_after(m, incident_tids, changed_faces, m_face_attribute);
}


bool TopoOffsetTetMesh::swap_edge_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_before(t)) {
        return false;
    }

    // do not swap edges on the surface for now
    if (is_edge_on_surface(t) || is_edge_on_offset(t) || is_edge_on_bbox(t)) {
        return false;
    }

    auto incident_tets = get_incident_tids_for_edge(t);
    if (incident_tets.size() != 3) {
        return false;
    }

    auto& cache = swap_edge_cache.local();
    cache.is_surface_flip = false;
    cache.is_offset_flip = false;

    // bbox edges are never swapped.
    if (is_edge_on_bbox(t)) {
        return false;
    }

    const auto& TA = m_tet_attribute;
    // cache.tet_tags = TA[incident_tets[0]].tag;

    // find tag with max count
    {
        std::map<CellTag, size_t> tag_count;
        for (const size_t l : incident_tets) {
            tag_count[TA[l].tag]++;
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
    if (is_edge_on_surface(t) || is_edge_on_offset(t)) {
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
        swap_edge_cache.local().changed_faces);

    return true;
}

bool TopoOffsetTetMesh::swap_edge_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_after(t)) {
        return false;
    }

    // after swap, t points to a face with 2 neighboring tets.
    auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");

    const auto& cache = swap_edge_cache.local();

    auto twotets = std::vector<Tuple>{{t, *oppo_tet}};
    auto max_energy = -1.0;
    for (auto& l : twotets) {
        if (is_inverted(l)) {
            return false;
        }
        const double q = get_quality(l);
        m_tet_attribute[l.tid(*this)].m_quality = q;
        max_energy = std::max(q, max_energy);

        m_tet_attribute[l.tid(*this)].tag = cache.tet_tags;
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

    if (cache.is_surface_flip || cache.is_offset_flip) {
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
        cnt_surface_swap++;
    }

    cnt_swap++;

    return true;
}

bool TopoOffsetTetMesh::swap_edge_44_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_44_before(t)) {
        return false;
    }

    const auto incident_tets = get_incident_tids_for_edge(t);
    if (incident_tets.size() != 4) {
        return false;
    }

    // unlike swap_edge (2-3/3-2), there is no surface-diagonal-flip case here: reject outright
    if (is_edge_on_surface(t) || is_edge_on_offset(t) || is_edge_on_bbox(t)) {
        return false;
    }

    auto& cache = swap_edge_cache.local();
    cache.is_surface_flip = false;
    cache.is_offset_flip = false;
    const auto& TA = m_tet_attribute;

    cache.tet_tags = TA[incident_tets[0]].tag;
    double max_energy = -1.0;
    for (const size_t l : incident_tets) {
        max_energy = std::max(TA[l].m_quality, max_energy);
    }
    cache.max_energy = max_energy;

    face_attribute_tracker(*this, incident_tets, m_face_attribute, cache.changed_faces);

    return true;
}

double TopoOffsetTetMesh::swap_edge_44_energy(
    const std::vector<std::array<size_t, 4>>& tets,
    const int op_case)
{
    double max_energy = -1;
    for (const auto& vids : tets) {
        if (is_inverted(vids)) {
            return std::numeric_limits<double>::max();
        }
        const double e = get_quality(vids);
        max_energy = std::max(max_energy, e);
    }
    return max_energy;
}

bool TopoOffsetTetMesh::swap_edge_44_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_44_after(t)) {
        return false;
    }

    const auto incident_tets = get_incident_tets_for_edge(t);

    double max_energy = -1.0;
    for (const Tuple& l : incident_tets) {
        if (is_inverted(l)) {
            return false;
        }
        const double q = get_quality(l);
        m_tet_attribute[l.tid(*this)].m_quality = q;
        max_energy = std::max(q, max_energy);

        m_tet_attribute[l.tid(*this)].tag = swap_edge_cache.local().tet_tags;
    }

    if (max_energy >= swap_edge_cache.local().max_energy) {
        return false;
    }

    tracker_assign_after(
        *this,
        incident_tets,
        swap_edge_cache.local().changed_faces,
        m_face_attribute);

    cnt_swap++;
    return true;
}

bool TopoOffsetTetMesh::swap_edge_56_before(const Tuple& t)
{
    if (!TetMesh::swap_edge_56_before(t)) {
        return false;
    }

    const auto incident_tets = get_incident_tids_for_edge(t);
    if (incident_tets.size() != 5) {
        return false;
    }
    if (is_edge_on_surface(t) || is_edge_on_offset(t) || is_edge_on_bbox(t)) {
        return false;
    }

    auto& cache = swap_edge_cache.local();
    cache.is_surface_flip = false;
    cache.is_offset_flip = false;
    const auto& TA = m_tet_attribute;

    cache.tet_tags = TA[incident_tets[0]].tag;
    double max_energy = -1.0;
    for (const size_t l : incident_tets) {
        max_energy = std::max(TA[l].m_quality, max_energy);
    }
    cache.max_energy = max_energy;

    face_attribute_tracker(*this, incident_tets, m_face_attribute, cache.changed_faces);

    return true;
}

double TopoOffsetTetMesh::swap_edge_56_energy(
    const std::vector<std::array<size_t, 4>>& tets,
    const int op_case)
{
    double max_energy = -1;
    for (const auto& vids : tets) {
        if (is_inverted(vids)) {
            return std::numeric_limits<double>::max();
        }
        const double e = get_quality(vids);
        max_energy = std::max(max_energy, e);
    }
    return max_energy;
}

bool TopoOffsetTetMesh::swap_edge_56_after(const Tuple& t)
{
    if (!TetMesh::swap_edge_56_after(t)) {
        return false;
    }

    // No need to check inversion or energy against a "before" baseline here: the operation
    // would already have been rejected via swap_edge_56_energy() if every orientation were
    // worse. The inversion check below is a final safety net, matching SimWildMesh.
    const auto e1 = get_incident_tids_for_edge(t);
    const auto e2 = get_incident_tids_for_edge(t.switch_edge(*this));
    const auto tids = wmtk::set_union(e1, e2);

    for (const size_t tid : tids) {
        const Tuple tet = tuple_from_tet(tid);
        if (is_inverted(tet)) {
            return false;
        }
        const double q = get_quality(tet);
        m_tet_attribute[tid].m_quality = q;
        m_tet_attribute[tid].tag = swap_edge_cache.local().tet_tags;
    }

    tracker_assign_after(*this, tids, swap_edge_cache.local().changed_faces, m_face_attribute);

    cnt_swap++;
    return true;
}

bool TopoOffsetTetMesh::swap_face_before(const Tuple& t)
{
    if (!TetMesh::swap_face_before(t)) {
        return false;
    }

    auto& cache = swap_edge_cache.local();
    cache.is_surface_flip = false;
    cache.is_offset_flip = false;

    const size_t fid = t.fid(*this);
    if (m_face_attribute[fid].m_is_surface_fs || m_face_attribute[fid].m_is_bbox_fs >= 0) {
        return false;
    }
    const auto oppo_tet = t.switch_tetrahedron(*this);
    assert(oppo_tet.has_value() && "Should not swap boundary.");

    const size_t t0 = t.tid(*this);
    const size_t t1 = oppo_tet.value().tid(*this);

    const auto& TA = m_tet_attribute;

    const double max_energy = std::max(TA[t0].m_quality, TA[t1].m_quality);

    // pre-compute energy for all 3 possible re-triangulations of the 2-tet bistellar flip; the
    // real connectivity change never moves any vertex, so this is exact, not an estimate
    {
        const auto t1_vids = oriented_tet_vids(t1);

        const size_t v0 = t.vid(*this);
        const size_t v1 = t.switch_vertex(*this).vid(*this);
        const size_t v2 = t.switch_edge(*this).switch_vertex(*this).vid(*this);
        const size_t v3 = t.switch_face(*this).switch_edge(*this).switch_vertex(*this).vid(*this);

        const std::array<size_t, 3> tri{{v0, v1, v2}};

        for (int i = 0; i < 3; i++) {
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

    cache.tet_tags = TA[t0].tag;

    const std::vector<size_t> twotets{t0, t1};
    face_attribute_tracker(*this, twotets, m_face_attribute, cache.changed_faces);
    return true;
}

bool TopoOffsetTetMesh::swap_face_after(const Tuple& t)
{
    if (!TetMesh::swap_face_after(t)) {
        return false;
    }

    const auto incident_tets = get_incident_tets_for_edge(t);

    for (const Tuple& l : incident_tets) {
        const double q = get_quality(l);
        m_tet_attribute[l.tid(*this)].m_quality = q;
        m_tet_attribute[l.tid(*this)].tag = swap_edge_cache.local().tet_tags;
    }

    tracker_assign_after(
        *this,
        incident_tets,
        swap_edge_cache.local().changed_faces,
        m_face_attribute);

    cnt_swap++;
    return true;
}

bool TopoOffsetTetMesh::offset_swap_normal_deviation_ok(
    const Tuple& face_abc,
    const Tuple& face_abd,
    size_t a,
    size_t b,
    size_t c,
    size_t d) const
{
    // pool the 4 target-normal samples from both current offset faces, matching the
    // reference's 8-sample pool (4 per triangle)
    std::vector<OffsetSurfaceSample> samples;
    for (const OffsetSurfaceSample& s : offset_surface_samples(face_abc)) samples.push_back(s);
    for (const OffsetSurfaceSample& s : offset_surface_samples(face_abd)) samples.push_back(s);

    const Vector3d pa = m_vertex_attribute[a].m_posf;
    const Vector3d pb = m_vertex_attribute[b].m_posf;
    const Vector3d pc = m_vertex_attribute[c].m_posf;
    const Vector3d pd = m_vertex_attribute[d].m_posf;

    // spread (max-min) of the angle between `dir` and every pooled sample, same formula as
    // collapse_normal_deviation()
    auto spread = [&samples](const Vector3d& dir) {
        double min_angle = std::numeric_limits<double>::max();
        double max_angle = std::numeric_limits<double>::lowest();
        for (const OffsetSurfaceSample& s : samples) {
            if (s.normal.squaredNorm() < 1e-20) continue; // degenerate: no normal direction
            const double dot = std::clamp(dir.dot(s.normal), -1., 1.);
            const double angle = (180. / M_PI) * std::acos(dot);
            min_angle = std::min(min_angle, angle);
            max_angle = std::max(max_angle, angle);
        }
        if (min_angle > max_angle) return 0.; // no samples found at all
        return max_angle - min_angle;
    };

    const double nd_old = spread((pb - pa).normalized()); // current diagonal (a,b)
    const double nd_new = spread((pd - pc).normalized()); // diagonal after the flip (c,d)

    if (nd_old >= m_max_normal_deviation_swap_max_deg ||
        nd_new >= m_max_normal_deviation_swap_max_deg) {
        // something might be off here, better don't flip
        return false;
    }

    // only reject a regression: an already-poor alignment doesn't block the flip
    if (nd_old < m_offset_params.max_normal_deviation_deg &&
        nd_new >= m_offset_params.max_normal_deviation_deg) {
        return false;
    }
    return true;
}

bool TopoOffsetTetMesh::prepare_surface_flip_32(
    const Tuple& t,
    const std::vector<size_t>& incident_tets)
{
    auto& cache = swap_edge_cache.local();

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
    int n_offset = 0;
    size_t c = 0, d = 0, e = 0; // vertex ids; c/d on surface, e not
    bool e_set = false;
    Tuple offset_face_c, offset_face_d; // captured only for the offset case, see below
    for (int k = 0; k < 3; ++k) {
        const auto [face_tuple, fid] = tuple_from_face(std::array<size_t, 3>{{a, b, ring[k]}});
        if (face_is_input(fid)) {
            if (n_surf == 0) {
                c = ring[k];
                cache.sf_face_attr = m_face_attribute[fid];
            } else if (n_surf == 1) {
                d = ring[k];
            } else {
                return false; // > 2 surface faces: non-manifold edge
            }
            ++n_surf;
        } else if (face_is_offset(fid)) {
            if (n_offset == 0) {
                c = ring[k];
                cache.sf_face_attr = m_face_attribute[fid];
                offset_face_c = face_tuple;
            } else if (n_offset == 1) {
                d = ring[k];
                offset_face_d = face_tuple;
            } else {
                return false; // > 2 offset faces: non-manifold edge
            }
            ++n_offset;
        } else {
            if (e_set) {
                return false; // > 1 non-surface face
            }
            e = ring[k];
            e_set = true;
        }
    }
    // either 2 surface faces and no offset faces, or 2 offset faces and no surface faces. No
    // mixed surface/offset flips.
    if (n_surf == 2 && n_offset != 0) {
        return false;
    }
    if (n_offset == 2 && n_surf != 0) {
        return false;
    }

    // The flip adds two surface faces (a,c,d),(b,c,d) on edge (c,d). If any
    // surface face is already incident to edge (c,d), the result is a
    // non-manifold surface edge (> 2 surface faces) -> reject. This counts the
    // incident surface faces directly and does NOT rely on the m_is_on_input
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

    // OffsetSwapInvariant analogue: without this, a swap can freely re-triangulate the offset
    // surface's diagonal even when doing so points the result away from the input complex's
    // implicit offset field, since nothing else here checks alignment with it (only the input
    // surface's own flip is protected, by the envelope containment check in swap_edge_after).
    if (n_offset == 2 &&
        !offset_swap_normal_deviation_ok(offset_face_c, offset_face_d, a, b, c, d)) {
        return false;
    }

    cache.is_surface_flip = (n_surf == 2);
    cache.is_offset_flip = (n_offset == 2);
    assert(cache.is_surface_flip != cache.is_offset_flip);
    cache.sf_a = a;
    cache.sf_b = b;
    cache.sf_c = c;
    cache.sf_d = d;
    cache.sf_e = e;
    return true;
}


void TopoOffsetTetMesh::swap_all_edges()
{
    // 2-3/3-2 flip only; see swap_all_edges_44()/swap_all_edges_56()/swap_all_faces() for the
    // other three, and swap_all_edges_all() for running this one together with 4-4 and 5-6.
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

void TopoOffsetTetMesh::swap_all_edges_44()
{
    std::vector<std::pair<std::string, Tuple>> all_ops = wmtk::parallel_collect_edge_ops(
        *this,
        NUM_THREADS,
        [](TopoOffsetTetMesh&, const Tuple& e, auto& out) { out.emplace_back("edge_swap_44", e); });

    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_edges;
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

void TopoOffsetTetMesh::swap_all_edges_56()
{
    std::vector<std::pair<std::string, Tuple>> all_ops = wmtk::parallel_collect_edge_ops(
        *this,
        NUM_THREADS,
        [](TopoOffsetTetMesh&, const Tuple& e, auto& out) { out.emplace_back("edge_swap_56", e); });

    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_edges;
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

void TopoOffsetTetMesh::swap_all_faces()
{
    // Seeded from canonical edge tuples, same as SimWildMesh::swap_all_faces(): each such
    // tuple is still anchored at *some* face, giving a first pass of coverage, and
    // renew_neighbor_tuples (wmtk::renewal_faces) re-enqueues genuinely face-anchored tuples
    // around every successful flip from then on.
    std::vector<std::pair<std::string, Tuple>> all_ops = wmtk::parallel_collect_edge_ops(
        *this,
        NUM_THREADS,
        [](TopoOffsetTetMesh&, const Tuple& e, auto& out) { out.emplace_back("face_swap", e); });

    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples = wmtk::renewal_faces;
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
            return m.try_set_face_mutex_two_ring(e, task_id);
        };
        executor.num_threads = NUM_THREADS;
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<TopoOffsetTetMesh>(wmtk::ExecutionPolicy::kSeq);
        setup_and_execute(executor);
    }
}

void TopoOffsetTetMesh::swap_all_edges_all()
{
    std::vector<std::pair<std::string, Tuple>> all_ops = wmtk::parallel_collect_edge_ops(
        *this,
        NUM_THREADS,
        [](TopoOffsetTetMesh&, const Tuple& e, auto& out) {
            out.emplace_back("edge_swap", e);
            out.emplace_back("edge_swap_44", e);
            out.emplace_back("edge_swap_56", e);
        });

    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples =
            [](const TopoOffsetTetMesh& m, const std::string&, const std::vector<Tuple>& newt) {
                std::vector<Tuple> new_edges;
                for (const Tuple& ti : newt) {
                    for (int j = 0; j < 6; ++j) {
                        new_edges.push_back(m.tuple_from_edge(ti.tid(m), j));
                    }
                }
                wmtk::unique_edge_tuples(m, new_edges);

                std::vector<std::pair<std::string, Tuple>> op_tups;
                op_tups.reserve(new_edges.size() * 3);
                for (const Tuple& loc : new_edges) {
                    op_tups.emplace_back("edge_swap", loc);
                    op_tups.emplace_back("edge_swap_44", loc);
                    op_tups.emplace_back("edge_swap_56", loc);
                }
                return op_tups;
            };
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
