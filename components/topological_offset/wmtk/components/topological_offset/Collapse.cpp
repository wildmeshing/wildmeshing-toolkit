
#include "TopoOffsetTetMesh.h"

#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/ParallelCollect.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>

namespace wmtk::components::topological_offset {

double TopoOffsetTetMesh::face_normal_deviation(const Tuple& f) const
{
    const std::array<Tuple, 3> fv = get_face_vertices(f);
    const Vector3d p_a = m_vertex_attribute[fv[0].vid(*this)].m_posf;
    const Vector3d p_b = m_vertex_attribute[fv[1].vid(*this)].m_posf;
    const Vector3d p_c = m_vertex_attribute[fv[2].vid(*this)].m_posf;

    const Vector3d ab = p_b - p_a;
    const Vector3d ac = p_c - p_a;
    const Vector3d cross = ab.cross(ac);
    const double cross_norm = cross.norm();
    if (cross_norm < 1e-12) return 0.; // degenerate triangle, nothing to measure
    const Vector3d face_normal = cross / cross_norm;

    // max over all 4 samples, not just the centroid: a face straddling a feature has samples
    // on both sides of it, and only the max catches the one that disagrees with the face's own
    // (necessarily flat) normal -- averaging or using a single sample would miss it.
    double max_dev = 0.;
    for (const OffsetSurfaceSample& s : offset_surface_samples(f)) {
        if (s.normal.squaredNorm() < 1e-20) continue; // degenerate: no normal direction
        // orientation independent: a triangle whose plane is parallel to the sample's implied
        // plane counts as aligned regardless of which way get_face_vertices() winds it
        const double c = std::clamp(face_normal.cross(s.normal).norm(), -1., 1.);
        max_dev = std::max(max_dev, (180. / M_PI) * std::asin(c));
    }
    return max_dev;
}

double TopoOffsetTetMesh::max_offset_surface_normal_deviation_at_vertex(size_t vid) const
{
    double max_nd = 0.;
    for (const Tuple& f : get_offset_surface_faces_for_vertex(tuple_from_vertex(vid))) {
        max_nd = std::max(max_nd, face_normal_deviation(f));
    }
    return max_nd;
}

double TopoOffsetTetMesh::collapse_normal_deviation(const Tuple& edge, size_t remove_vid) const
{
    const Vector3d p0 = m_vertex_attribute[edge.vid(*this)].m_posf;
    const Vector3d p1 = m_vertex_attribute[edge.switch_vertex(*this).vid(*this)].m_posf;
    const Vector3d e_dir = (p1 - p0).normalized();

    double min_angle = std::numeric_limits<double>::max();
    double max_angle = std::numeric_limits<double>::lowest();
    for (const Tuple& f : get_offset_surface_faces_for_vertex(tuple_from_vertex(remove_vid))) {
        for (const OffsetSurfaceSample& s : offset_surface_samples(f)) {
            if (s.normal.squaredNorm() < 1e-20) continue; // degenerate: no normal direction
            // orientation dependent (0-180): unlike face_normal_deviation, the edge direction
            // has a genuine sign, so a sample pointing "with" vs "against" it are different
            const double dot = std::clamp(e_dir.dot(s.normal), -1., 1.);
            const double angle = (180. / M_PI) * std::acos(dot);
            min_angle = std::min(min_angle, angle);
            max_angle = std::max(max_angle, angle);
        }
    }
    if (min_angle > max_angle) return 0.; // no samples found at all
    return max_angle - min_angle;
}

bool TopoOffsetTetMesh::collapse_edge_before(const Tuple& loc)
{
    auto& cache = edge_collapse_cache.local();
    const auto& VA = m_vertex_attribute;

    cache.changed_faces.clear();
    cache.changed_tids.clear();
    cache.changed_energies.clear();
    cache.surface_faces.clear();
    cache.offset_faces.clear();
    cache.boundary_edges.clear();

    const size_t v1_id = loc.vid(*this); // removed by the collapse
    Tuple loc1 = loc.switch_vertex(*this);
    const size_t v2_id = loc1.vid(*this); // survives, and keeps its position

    cache.v1_id = v1_id;
    cache.v2_id = v2_id;

    cache.edge_length = (VA[v1_id].m_posf - VA[v2_id].m_posf).norm();

    // if (VA[v1_id].m_is_on_surface || VA[v2_id].m_is_on_surface) {
    //     // don't touch the input surface
    //     return false;
    // }

    // length, similar to SimWild: only collapse edges shorter than the target-length-derived
    // cutoff (m_params.collapsing_l2, set in Parameters::init() from length/length_rel),
    // scaled by the sizing field so a refined region resists collapsing back down
    {
        const double sizing_ratio =
            (VA[v1_id].m_sizing_scalar + VA[v2_id].m_sizing_scalar) / 2.;
        if ((VA[v1_id].m_posf - VA[v2_id].m_posf).squaredNorm() >
            m_params.collapsing_l2 * sizing_ratio * sizing_ratio) {
            return false;
        }
    }

    ///check if on bbox/surface/boundary
    // bbox
    if (!VA[v1_id].on_bbox_faces.empty()) {
        if (VA[v2_id].on_bbox_faces.size() < VA[v1_id].on_bbox_faces.size()) return false;
        for (int on_bbox : VA[v1_id].on_bbox_faces)
            if (std::find(
                    VA[v2_id].on_bbox_faces.begin(),
                    VA[v2_id].on_bbox_faces.end(),
                    on_bbox) == VA[v2_id].on_bbox_faces.end()) {
                return false;
            }
    }

    // surface
    if (cache.edge_length > 0 && VA[v1_id].m_is_on_surface) {
        if (!VA[v2_id].m_is_on_surface) {
            // do not collapse away from surface
            return false;
        }
    }
    if (cache.edge_length > 0 && VA[v1_id].m_is_on_offset) {
        if (!VA[v2_id].m_is_on_offset) {
            // do not collapse away from offset
            return false;
        }
    }

    // open boundary
    if (cache.edge_length > 0 && VA[v1_id].m_order == 2) {
        if (VA[v2_id].m_order < 2) {
            return false;
        }
    }

    // OffsetCollapseBeforeInvariant analogue: don't collapse if the offset-target normal field
    // sampled around the survivor disagrees with itself (relative to the collapse direction)
    // by more than the threshold -- that disagreement is the signature of a feature edge
    // nearby, and collapsing across it would flatten/cut through the feature.
    if (collapse_normal_deviation(loc, v1_id) >= m_params.max_normal_deviation_deg) {
        return false;
    }

    // NormalDeviationAfterInvariant analogue setup: remember how bad the offset surface
    // already was around this edge, so collapse_edge_after() only blocks a collapse that
    // makes a *good* patch worse, not one that was already over the threshold.
    cache.nd_before = std::max(
        max_offset_surface_normal_deviation_at_vertex(v1_id),
        max_offset_surface_normal_deviation_at_vertex(v2_id));

    const auto n1_locs = get_one_ring_tids_for_vertex(loc);

    cache.changed_tids.reserve(n1_locs.size());
    cache.max_energy = 0;
    for (const size_t& tid : n1_locs) {
        const double q = m_tet_attribute.at(tid).m_quality;
        cache.max_energy = std::max(cache.max_energy, q);
        const auto vs = oriented_tet_vids(tid);
        if (vs[0] != v2_id && vs[1] != v2_id && vs[2] != v2_id && vs[3] != v2_id) {
            cache.changed_tids.emplace_back(tid);
        }
    }

    // pre-compute after-collapse energies
    cache.changed_energies.reserve(cache.changed_tids.size());
    for (const size_t tid : cache.changed_tids) {
        std::array<size_t, 4> vs = oriented_tet_vids(tid);
        for (size_t i = 0; i < 4; ++i) {
            if (vs[i] == v1_id) {
                vs[i] = v2_id;
                break;
            }
        }

        if (is_inverted(vs)) {
            return false;
        }
        double q = get_quality(vs);
        if (q > cache.max_energy) {
            return false;
        }
        cache.changed_energies.emplace_back(q);
    }
    assert(cache.changed_energies.size() == cache.changed_tids.size());

    // if (m_params.perform_sanity_checks) {
    //     if (!link_condition(loc)) {
    //         log_and_throw_error("link condition failed for edge ({}, {})", v1_id, v2_id);
    //     }
    // }

    const auto n12_locs = get_incident_tids_for_edge(loc);
    for (const size_t& tid : n12_locs) {
        auto vs = oriented_tet_vids(tid);
        std::array<size_t, 3> f_vids = {{v1_id, 0, 0}};
        int cnt = 1;
        // get the two vertices that are not v1/v2, i.e., the edge-link vertices.
        for (int j = 0; j < 4; j++) {
            if (vs[j] != v1_id && vs[j] != v2_id) {
                f_vids[cnt] = vs[j];
                cnt++;
            }
        }
        auto [_1, global_fid1] = tuple_from_face(f_vids);
        auto [_2, global_fid2] = tuple_from_face({{v2_id, f_vids[1], f_vids[2]}});
        auto f_attr = m_face_attribute.at(global_fid1);
        f_attr.merge(m_face_attribute.at(global_fid2));
        cache.changed_faces.push_back(std::make_pair(f_attr, f_vids));
    }

    if (VA[v1_id].m_is_on_surface) {
        simplex::SimplexCollection fs = get_surface_faces_for_vertex(v1_id);

        cache.surface_faces.reserve(fs.faces().size());
        cache.offset_faces.reserve(fs.faces().size());
        for (auto& f : fs.faces()) {
            const simplex::Edge e_opp = f.opposite_edge(v1_id);
            const size_t e0 = e_opp.vertices()[0];
            const size_t e1 = e_opp.vertices()[1];
            if (e0 == v2_id || e1 == v2_id) {
                continue;
            }
            const auto [f_tuple, fid] = tuple_from_face(f.vertices());
            if (m_face_attribute.at(fid).m_is_offset_fs) {
                cache.offset_faces.push_back({{v2_id, e0, e1}});
            } else if (m_face_attribute.at(fid).m_is_surface_fs) {
                cache.surface_faces.push_back({{v2_id, e0, e1}});
            } else {
                log_and_throw_error("Surface face {} is neither offset nor surface", fid);
            }
            // cache.surface_faces.push_back({{v2_id, e0, e1}});
        }

        std::vector<std::array<size_t, 2>> bs;
        // iterate through all faces inicdent to v1
        for (const size_t& tid : n1_locs) {
            const auto vs = oriented_tet_vids(tid);

            int j_v1 = -1;
            for (int j = 0; j < 4; j++) {
                const size_t vid = vs[j];
                if (vid == v1_id) {
                    j_v1 = j;
                }
            }

            for (int k = 0; k < 3; k++) {
                const size_t va = vs[(j_v1 + 1 + k) % 4];
                const size_t vb = vs[(j_v1 + 1 + (k + 1) % 3) % 4];
                if ((!VA[va].m_is_on_surface || !VA[vb].m_is_on_surface)) {
                    continue;
                }
                const auto [f_tuple, fid] = tuple_from_face({{v1_id, va, vb}});
                if (!m_face_attribute.at(fid).m_is_surface_fs) {
                    // check if this face is actually on the surface
                    continue;
                }
                if (va != v2_id) { // ignore collapsing edge (v1,v2)
                    std::array<size_t, 2> ba = {{v1_id, va}};
                    if (is_order_2_edge(ba)) {
                        ba[0] = v2_id; // replace v1 with v2 for check in `after` function
                        std::sort(ba.begin(), ba.end());
                        bs.push_back(ba);
                    }
                }
                if (vb != v2_id) { // ignore collapsing edge (v1,v2)
                    std::array<size_t, 2> bb = {{v1_id, vb}};
                    if (is_order_2_edge(bb)) {
                        bb[0] = v2_id; // replace v1 with v2 for check in `after` function
                        std::sort(bb.begin(), bb.end());
                        bs.push_back(bb);
                    }
                }
            }
        }
        wmtk::vector_unique(bs);
        cache.boundary_edges = bs;
    }

    if ((VA[v1_id].m_is_on_surface || VA[v1_id].m_is_on_offset) &&
        (VA[v2_id].m_is_on_surface || VA[v2_id].m_is_on_offset)) {
        if (!substructure_link_condition(loc)) {
            return false;
        }
    }

    return true;
}

bool TopoOffsetTetMesh::collapse_edge_after(const Tuple& loc)
{
    auto& VA = m_vertex_attribute;
    auto& cache = edge_collapse_cache.local();
    const size_t v1_id = cache.v1_id;
    const size_t v2_id = cache.v2_id;

    assert(v2_id == loc.vid(*this));

    if (!TetMesh::collapse_edge_after(loc)) {
        return false;
    }

    // open boundary - must be set before checking for open boundary
    VA[v2_id].m_order = std::max(VA.at(v1_id).m_order, VA.at(v2_id).m_order);

    // NormalDeviationAfterInvariant analogue: only reject a move that degrades an
    // already-good offset surface patch -- if it was already over the threshold before, don't
    // block a collapse from fixing (or merely not fixing) it.
    if (cache.nd_before < m_params.max_normal_deviation_deg) {
        const double nd_after = max_offset_surface_normal_deviation_at_vertex(loc.vid(*this));
        if (nd_after >= m_params.max_normal_deviation_deg) {
            return false;
        }
    }

    // surface
    // and order 2 edges
    if (cache.edge_length > 0) {
        for (auto& vids : cache.surface_faces) {
            // surface envelope
            bool is_out = m_envelope->is_outside(
                {{VA.at(vids[0]).m_posf, VA.at(vids[1]).m_posf, VA.at(vids[2]).m_posf}});
            if (is_out) {
                return false;
            }
        }
        // for (const auto& vids : cache.boundary_edges) {
        //     std::array<Vector3d, 2> pts{{VA.at(vids[0]).m_posf, VA.at(vids[1]).m_posf}};
        //     if (m_order_2_edge_envelope->is_outside(pts)) {
        //         return false;
        //     }
        // }
    }

    //// update attrs
    // tet attr
    for (int i = 0; i < cache.changed_tids.size(); i++) {
        m_tet_attribute[cache.changed_tids[i]].m_quality = cache.changed_energies[i];
    }
    // vertex attr
    VA[v2_id].m_is_on_surface = VA.at(v1_id).m_is_on_surface || VA.at(v2_id).m_is_on_surface;

    // no need to update on_bbox_faces
    // face attr
    for (auto& info : cache.changed_faces) {
        auto& f_attr = info.first;
        auto& old_vids = info.second;
        //
        auto [_, global_fid] = tuple_from_face({{v2_id, old_vids[1], old_vids[2]}});
        if (global_fid == -1) {
            return false;
        }
        m_face_attribute[global_fid] = f_attr;
    }

    return true;
}

void TopoOffsetTetMesh::collapse_all_edges()
{
    // mirrors SimWild::collapse_all_edges (EdgeCollapsing.cpp): collect both directions of
    // every edge, then let the executor sort out which one collapse_edge_before() accepts.
    std::vector<std::pair<std::string, Tuple>> all_ops = wmtk::parallel_collect_edge_ops(
        *this,
        NUM_THREADS,
        [](TopoOffsetTetMesh& m, const Tuple& e, auto& out) {
            out.emplace_back("edge_collapse", e);
            out.emplace_back("edge_collapse", e.switch_vertex(m));
        });

    auto setup_and_execute = [&](auto& executor) {
        // re-try the (possibly new) edges around a successful collapse, in both directions
        executor.renew_neighbor_tuples = [](const auto& m, auto op, const auto& newts) {
            std::vector<std::pair<std::string, Tuple>> op_tups;
            for (const Tuple& t : newts) {
                op_tups.emplace_back(op, t);
                op_tups.emplace_back(op, t.switch_vertex(m));
            }
            return op_tups;
        };
        // shortest edges first: squared length is monotonic with length and avoids the sqrt
        executor.priority = [](const TopoOffsetTetMesh& m, wmtk::Op, const Tuple& t) {
            const size_t v0 = t.vid(m);
            const size_t v1 = t.switch_vertex(m).vid(m);
            return -(m.m_vertex_attribute[v0].m_posf - m.m_vertex_attribute[v1].m_posf)
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
