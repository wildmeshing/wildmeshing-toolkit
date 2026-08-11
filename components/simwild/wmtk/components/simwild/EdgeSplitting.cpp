#include <limits>
#include "SimWildMesh.h"

#include <igl/Timer.h>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/ParallelCollect.hpp>
#include <wmtk/utils/RunPass.hpp>

namespace wmtk::components::simwild {

void SimWildMesh::split_all_edges()
{
    igl::Timer timer;
    double time;
    m_force_split_count = 0;
    m_exact_split_count = 0;
    timer.start();
    auto collect_all_ops = wmtk::parallel_collect_edge_ops(
        *this,
        NUM_THREADS,
        [](SimWildMesh&, const Tuple& e, auto& out) { out.emplace_back("edge_split", e); });
    time = timer.getElapsedTime();
    wmtk::logger().info("edge split prepare time: {:.4}s", time);
    wmtk::run_pass(
        *this,
        wmtk::PassLock::EdgeTwoRing,
        "edge split operation",
        [&](auto& executor, auto& mesh) {
            executor.renew_neighbor_tuples = wmtk::renewal_edges;

            executor.priority = [&](const wmtk::TetOptimizerMesh& m,
                                    std::string op,
                                    const Tuple& t) { return m.get_length2(t); };
            executor.is_weight_up_to_date = [&](const wmtk::TetOptimizerMesh& m,
                                                const std::tuple<double, std::string, Tuple>& ele) {
                auto [weight, op, tup] = ele;
                auto length = m.get_length2(tup);
                if (length != weight) {
                    return false;
                }
                //
                size_t v1_id = tup.vid(*this);
                size_t v2_id = tup.switch_vertex(*this).vid(*this);
                // Force-split: a worst tet's longest edge (queued by
                // refine_sizing_around_worst when the max energy stalls) is split once
                // regardless of the length gate, to unstick a sliver without changing the
                // sizing field. The new midpoint is not in m_force_split_edges, so the
                // two halves are NOT force-split again -- exactly one split per edge.
                if (is_force_split_edge(v1_id, v2_id)) {
                    return true;
                }
                double sizing_ratio = (m_vertex_attribute[v1_id].m_sizing_scalar +
                                       m_vertex_attribute[v2_id].m_sizing_scalar) /
                                      2;
                if (length < m_params.splitting_l2 * sizing_ratio * sizing_ratio) {
                    return false;
                }
                return true;
            };
            // Retry a failed split only where the mesh actually changed this round
            // (dirty-epoch localized retry), as tetwild does.
            wmtk::run_localized_to_convergence(mesh, executor, collect_all_ops);
        });
    if (m_force_split_count > 0) {
        wmtk::logger().info(
            "[force-split] {} worst-tet longest edges force-split",
            m_force_split_count);
    }
    if (m_exact_split_count > 0) {
        wmtk::logger().info(
            "{} splits fell back to the exact rational midpoint",
            m_exact_split_count);
    }
    // Consumed: the queued force-split edges no longer exist after this pass
    m_force_split_edges.clear();
}

bool SimWildMesh::split_edge_before(const Tuple& loc0)
{
    auto& cache = split_cache.local();

    cache.changed_faces.clear();
    cache.tets.clear();

    cache.v1_id = loc0.vid(*this);
    auto loc1 = loc0.switch_vertex(*this);
    cache.v2_id = loc1.vid(*this);

    cache.max_quality_before = 0.;
    for (const size_t tid : get_incident_tids_for_edge(loc0)) {
        cache.max_quality_before =
            std::max(cache.max_quality_before, m_tet_attribute[tid].m_quality);
    }

    cache.is_edge_on_surface = is_edge_on_surface(loc0);

    // todo: can be optimized
    if (m_params.preserve_topology) {
        cache.is_edge_open_boundary = cache.is_edge_on_surface && is_order_2_edge(loc0);
    } else {
        cache.is_edge_open_boundary = false;
    }

    /// save face track info
    auto comp = [](const std::pair<FaceAttributes, std::array<size_t, 3>>& v1,
                   const std::pair<FaceAttributes, std::array<size_t, 3>>& v2) {
        return v1.second < v2.second;
    };
    auto is_equal = [](const std::pair<FaceAttributes, std::array<size_t, 3>>& v1,
                       const std::pair<FaceAttributes, std::array<size_t, 3>>& v2) {
        return v1.second == v2.second;
    };

    auto tets = get_incident_tets_for_edge(loc0);
    for (auto& t : tets) {
        auto vs = oriented_tet_vertices(t);
        for (int j = 0; j < 4; j++) {
            std::array<size_t, 3> f_vids = {{
                vs[(j + 1) % 4].vid(*this),
                vs[(j + 2) % 4].vid(*this),
                vs[(j + 3) % 4].vid(*this),
            }}; // todo: speedup
            std::sort(f_vids.begin(), f_vids.end());
            auto [_, global_fid] = tuple_from_face(f_vids);
            cache.changed_faces.push_back(std::make_pair(m_face_attribute[global_fid], f_vids));
        }
    }
    wmtk::vector_unique(cache.changed_faces, comp, is_equal);

    // store tet attributes
    const simplex::Edge edge(cache.v1_id, cache.v2_id);
    for (const Tuple& t : tets) {
        const simplex::Tet tet = simplex_from_tet(t);
        const simplex::Edge opp = tet.opposite_edge(edge);
        // if (m_tet_attribute.at(t.tid(*this)).tags.size() == 0) {
        //    log_and_throw_error("No tags in tet {}", t.tid(*this)); // for debugging
        //}
        cache.tets[opp] = m_tet_attribute.at(t.tid(*this));
    }

    return true;
}

bool SimWildMesh::split_edge_after(const Tuple& loc)
{ // input: locs pointing to a list of tets and v_id
    if (!TetMesh::split_edge_after(
            loc)) // note: call from super class, cannot be done with pure virtual classes
        return false;

    const std::vector<Tuple> locs = get_one_ring_tets_for_vertex(loc);
    const size_t v_id = loc.vid(*this);

    auto& cache = split_cache.local();
    cache.v_new = v_id;

    const size_t v1_id = cache.v1_id;
    const size_t v2_id = cache.v2_id;

    /// check inversion & rounding
    auto& p = m_vertex_attribute[v_id].m_posf;
    p = (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2;
    m_vertex_attribute[v_id].m_is_rounded = true;

    // this has to be done before the inversion check
    m_vertex_attribute[v_id].m_pos = to_rational(p);

    for (const Tuple& t : locs) {
        if (is_inverted(t)) {
            m_vertex_attribute[v_id].m_is_rounded = false;
            break;
        }
    }
    if (is_force_split_edge(v1_id, v2_id)) {
        std::atomic_ref<size_t>(m_force_split_count).fetch_add(1, std::memory_order_relaxed);
    }
    if (!m_vertex_attribute[v_id].m_is_rounded) {
        std::atomic_ref<size_t>(m_exact_split_count).fetch_add(1, std::memory_order_relaxed);
        m_vertex_attribute[v_id].m_pos =
            (m_vertex_attribute[v1_id].m_pos + m_vertex_attribute[v2_id].m_pos) / 2;
        p = to_double(m_vertex_attribute[v_id].m_pos);
        // Guard against a pre-existing inverted incident tet: re-check in exact
        // arithmetic (un-rounded v_id => is_inverted uses the rational path).
        for (const Tuple& t : locs) {
            if (is_inverted(t)) {
                return false;
            }
        }
        // This split keeps an un-rounded vertex, so the sweep must not skip the next pass.
        // Set after the rollback checks above, which leave the mesh unchanged.
        m_all_rounded.store(false, std::memory_order_relaxed);
    }

    // If a Voronoi split function is set, binary-search vmid onto its zero-crossing.
    // p0 stays on the negative side, p1 on the positive side.
    //
    // Skipped for an un-rounded vertex: the exact midpoint is then the only position known to
    // keep every incident tet valid, and this search only considers doubles -- including the
    // plain double midpoint it reverts to, which is the position that just inverted. The 2D
    // twin carries the same guard.
    if (m_voronoi_split_fn && m_vertex_attribute[v_id].m_is_rounded) {
        Vector3d p0 = m_vertex_attribute[v1_id].m_posf;
        Vector3d p1 = m_vertex_attribute[v2_id].m_posf;
        if (m_voronoi_split_fn(p0) >= 0) {
            std::swap(p0, p1); // ensure p0 is negative side
        }
        for (int i = 0; i < 20; ++i) {
            p = 0.5 * (p0 + p1);
            m_vertex_attribute[v_id].m_pos = to_rational(p);
            bool inv = false;
            for (const Tuple& t : locs) {
                if (is_inverted(t)) {
                    inv = true;
                    break;
                }
            }
            if (inv || (p1 - p0).squaredNorm() < 1e-20) {
                break;
            }
            if (m_voronoi_split_fn(p) < 0) {
                p0 = p;
            } else {
                p1 = p;
            }
        }
        // final inversion guard: revert to midpoint if needed
        bool inv = false;
        for (const Tuple& t : locs) {
            if (is_inverted(t)) {
                inv = true;
                logger().warn(
                    "Voronoi split resulted in inversion, reverting to midpoint. Iteration: {}",
                    m_debug_print_counter++);
                break;
            }
        }
        if (inv) {
            p = (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2;
            m_vertex_attribute[v_id].m_pos = to_rational(p);
        }
    }

    // update tet attributes
    {
        // v1 - v_new
        const auto tets1 = get_incident_tets_for_edge(v1_id, v_id);
        const simplex::Edge edge1(v1_id, v_id);
        for (const Tuple& t : tets1) {
            const simplex::Tet tet = simplex_from_tet(t);
            const simplex::Edge opp = tet.opposite_edge(edge1);
            m_tet_attribute[t.tid(*this)] = cache.tets[opp];
        }
        // v2 - v_new
        const auto tets2 = get_incident_tets_for_edge(v2_id, v_id);
        const simplex::Edge edge2(v2_id, v_id);
        for (const Tuple& t : tets2) {
            const simplex::Tet tet = simplex_from_tet(t);
            const simplex::Edge opp = tet.opposite_edge(edge2);
            m_tet_attribute[t.tid(*this)] = cache.tets[opp];
        }
        assert(tets1.size() + tets2.size() == locs.size());
    }

    /// update quality
    //
    // A split is otherwise unconditional: it checks orientation and the envelope but never
    // quality. That is right for a length-driven split of a long, well-behaved edge and
    // wrong for the force-split of a stalled sliver's longest edge, where the midpoint can
    // land essentially on the opposite edge and leave a correctly-oriented element whose
    // area/volume is too small for AMIPS -- get_quality then returns MAX_ENERGY, and every
    // control decision that divides by the max energy is meaningless from then on. Refuse
    // to be the operation that creates one; subdividing an already-degenerate region is
    // still allowed, so a stuck region can keep being refined. See tetwild's EdgeSplitting
    // for the measurement this comes from.
    double max_quality_after = 0.;
    for (const Tuple& loc : locs) {
        max_quality_after = std::max(max_quality_after, get_quality(loc));
    }
    if (max_quality_after >= MAX_ENERGY && cache.max_quality_before < MAX_ENERGY) {
        return false;
    }
    for (const Tuple& loc : locs) {
        m_tet_attribute[loc.tid(*this)].m_quality = get_quality(loc);
    }

    /// containment: the new surface triangles must stay inside the envelope
    //
    // Collapse and the swaps test this; split did not, and split is the operation that
    // CREATES surface vertices. It puts the new one at the chord midpoint, which on a curved
    // region lies off the surface by about L^2/8R, and nothing afterwards is obliged to
    // bring it back -- so the tracked surface walks outward one subdivision at a time.
    //
    // Measured on the tetwild counterpart of this hole (Thingi10K 243014, exact envelope,
    // serial): 19 of 24702 surface vertices outside the envelope by iteration 12, while the
    // end-of-run Hausdorff check still reported "inside the envelope (as expected)" -- that
    // check samples by AREA and is blind to violations on vanishingly small elements.
    if (cache.is_edge_on_surface) {
        const auto& VA = m_vertex_attribute;
        for (const auto& info : cache.changed_faces) {
            if (!info.first.m_is_surface_fs) continue;
            const auto& old_vids = info.second;
            size_t other = std::numeric_limits<size_t>::max();
            int n_shared = 0;
            for (int j = 0; j < 3; j++) {
                if (old_vids[j] == v1_id || old_vids[j] == v2_id) {
                    ++n_shared;
                } else {
                    other = old_vids[j];
                }
            }
            if (n_shared != 2) continue;
            if (m_envelope->is_outside(
                    {{VA[v1_id].m_posf, VA[v_id].m_posf, VA[other].m_posf}})) {
                return false;
            }
            if (m_envelope->is_outside(
                    {{VA[v2_id].m_posf, VA[v_id].m_posf, VA[other].m_posf}})) {
                return false;
            }
        }
    }

    /// update vertex attribute
    // bbox
    m_vertex_attribute[v_id].on_bbox_faces = wmtk::set_intersection(
        m_vertex_attribute[v1_id].on_bbox_faces,
        m_vertex_attribute[v2_id].on_bbox_faces);


    // surface
    m_vertex_attribute[v_id].m_is_on_surface = cache.is_edge_on_surface;
    if (m_params.preserve_topology) {
        if (cache.is_edge_on_surface) {
            m_vertex_attribute[v_id].m_order = 1;
        } else {
            m_vertex_attribute[v_id].m_order = 0;
        }
        if (cache.is_edge_open_boundary) {
            m_vertex_attribute[v_id].m_order = 2;
        }
    }

    /// update face attribute
    // add new and erase old
    for (auto& info : cache.changed_faces) {
        auto& f_attr = info.first;
        auto& old_vids = info.second;
        std::vector<int> j_vn;
        for (int j = 0; j < 3; j++) {
            if (old_vids[j] != v1_id && old_vids[j] != v2_id) {
                j_vn.push_back(j);
            }
        }
        if (j_vn.size() == 1) {
            auto [_1, global_fid1] = tuple_from_face({{v1_id, v_id, old_vids[j_vn[0]]}});
            m_face_attribute[global_fid1] = f_attr;
            auto [_2, global_fid2] = tuple_from_face({{v2_id, v_id, old_vids[j_vn[0]]}});
            m_face_attribute[global_fid2] = f_attr;
        } else { // j_vn.size() == 2
            auto [_, global_fid] = tuple_from_face(old_vids);
            m_face_attribute[global_fid] = f_attr;
            //
            auto [_2, global_fid2] = tuple_from_face(
                {{old_vids[j_vn[0]], old_vids[j_vn[1]], v_id}}); // todo: avoid dup comp
            m_face_attribute[global_fid2].reset();
        }
    }

    m_vertex_attribute[v_id].partition_id = m_vertex_attribute[v1_id].partition_id;
    m_vertex_attribute[v_id].m_sizing_scalar =
        (m_vertex_attribute[v1_id].m_sizing_scalar + m_vertex_attribute[v2_id].m_sizing_scalar) / 2;

    // if (m_vertex_attribute[v_id].m_is_on_surface) {
    //     if (!check_vertex_param_type()) {
    //         std::cout << v1_id << " " << v2_id << std::endl;
    //         for (auto vp : m_vertex_attribute[v1_id].face_nearly_param_type_with_ineffective)
    //             std::cout << vp << " ";
    //         std::cout << std::endl;
    //         for (auto vp : m_vertex_attribute[v2_id].face_nearly_param_type_with_ineffective)
    //             std::cout << vp << " ";
    //         std::cout << std::endl;
    //         output_faces("bug_surface_miss_param_after_split.obj", [](auto& f) {
    //             return f.m_is_surface_fs;
    //         });
    //         // exit(0);
    //     }
    // }

    return true;
}

} // namespace wmtk::components::simwild
