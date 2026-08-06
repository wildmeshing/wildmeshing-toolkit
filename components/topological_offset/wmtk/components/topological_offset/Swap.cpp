
#include "TopoOffsetTetMesh.h"

#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/ParallelCollect.hpp>

namespace wmtk::components::topological_offset {

void face_attribute_tracker(
    const TetMesh& m,
    const std::vector<size_t>& incident_tets,
    const TopoOffsetTetMesh::FaceAttCol& m_face_attribute,
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
    const std::map<std::array<size_t, 3>, FaceAttributes>& changed_faces,
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
        } else if (m_face_attribute[fid].m_is_offset_fs) {
            if (n_offset == 0) {
                c = ring[k];
                cache.sf_face_attr = m_face_attribute[fid];
            } else if (n_offset == 1) {
                d = ring[k];
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
