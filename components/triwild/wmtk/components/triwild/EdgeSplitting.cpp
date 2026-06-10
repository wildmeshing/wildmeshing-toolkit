#include "TriWildMesh.h"

#include <igl/Timer.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::triwild {

void TriWildMesh::split_all_edges()
{
    igl::Timer timer;
    double time;
    timer.start();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (const Tuple& loc : get_edges()) {
        collect_all_ops.emplace_back("edge_split", loc);
    }
    time = timer.getElapsedTime();
    logger().info("edge split prepare time: {:.4}s", time);
    auto setup_and_execute = [&](auto& executor) {
        executor.renew_neighbor_tuples =
            [](const TriWildMesh& m, std::string op, const auto& newts) {
                std::vector<std::pair<std::string, TriMesh::Tuple>> op_tups;
                for (const auto& t : newts) {
                    op_tups.emplace_back(op, t);
                    op_tups.emplace_back(op, t.switch_edge(m));
                    op_tups.emplace_back(op, t.switch_vertex(m).switch_edge(m));
                }
                return op_tups;
            };

        executor.priority = [&](const TriWildMesh& m, std::string op, const Tuple& t) {
            return m.get_length2(t);
        };
        executor.num_threads = NUM_THREADS;
        executor.is_weight_up_to_date = [&](const TriWildMesh& m, const auto& ele) {
            auto [weight, op, tup] = ele;
            auto length = m.get_length2(tup);
            if (length != weight) {
                return false;
            }
            //
            size_t v1_id = tup.vid(*this);
            size_t v2_id = tup.switch_vertex(*this).vid(*this);
            const auto& VA = m_vertex_attribute;
            double sizing_ratio = 0.5 * (VA[v1_id].m_sizing_scalar + VA[v2_id].m_sizing_scalar);
            if (length < m_params.splitting_l2 * sizing_ratio * sizing_ratio) {
                return false;
            }
            return true;
        };
        executor(*this, collect_all_ops);
    };
    if (NUM_THREADS > 0) {
        timer.start();
        auto executor = ExecutePass<TriWildMesh>(ExecutionPolicy::kPartition);
        executor.lock_vertices = [&](auto& m, const auto& e, int task_id) -> bool {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge split operation time parallel: {:.4}s", time);
    } else {
        timer.start();
        auto executor = ExecutePass<TriWildMesh>(ExecutionPolicy::kSeq);
        setup_and_execute(executor);
        time = timer.getElapsedTime();
        wmtk::logger().info("edge split operation time serial: {:.4}s", time);
    }
}

bool TriWildMesh::split_edge_before(const Tuple& loc0)
{
    auto& cache = split_cache.local();

    cache.changed_edges.clear();
    cache.faces.clear();

    cache.v1_id = loc0.vid(*this);
    cache.v2_id = loc0.switch_vertex(*this).vid(*this);

    cache.old_e_attrs = m_edge_attribute[loc0.eid(*this)];

    const simplex::Edge edge(cache.v1_id, cache.v2_id);

    const auto faces = get_incident_fids_for_edge(loc0);
    for (const size_t fid : faces) {
        auto vs = oriented_tri_vids(fid);
        for (int j = 0; j < 3; j++) {
            const simplex::Edge e(vs[j], vs[(j + 1) % 3]);
            if (e == edge) {
                continue;
            }
            if (cache.changed_edges.count(e) != 0) {
                continue;
            }
            auto [_, eid] = tuple_from_edge(e.vertices());
            cache.changed_edges[e] = m_edge_attribute[eid];
        }
    }

    // store tet attributes
    for (const size_t fid : faces) {
        const simplex::Face face = simplex_from_face(fid);
        const size_t opp = face.opposite_vertex(edge).id();
        cache.faces[opp] = m_face_attribute.at(fid);
    }

    return true;
}

bool TriWildMesh::split_edge_after(const Tuple& loc)
{
    if (!TriMesh::split_edge_after(
            loc)) // note: call from super class, cannot be done with pure virtual classes
        return false;

    const std::vector<Tuple> locs = get_one_ring_tris_for_vertex(loc.switch_vertex(*this));
    const size_t v_id = loc.switch_vertex(*this).vid(*this);

    auto& cache = split_cache.local();

    const size_t v1_id = cache.v1_id;
    const size_t v2_id = cache.v2_id;

    /// check inversion & rounding
    auto& p = m_vertex_attribute[v_id].m_posf;
    p = (m_vertex_attribute[v1_id].m_posf + m_vertex_attribute[v2_id].m_posf) / 2;
    m_vertex_attribute[v_id].m_is_rounded = true;

    // this has to be done before the inversion check
    m_vertex_attribute[v_id].m_pos = to_rational(p);

    for (auto& loc : locs) {
        if (is_inverted(loc)) {
            m_vertex_attribute[v_id].m_is_rounded = false;
            break;
        }
    }
    if (!m_vertex_attribute[v_id].m_is_rounded) {
        m_vertex_attribute[v_id].m_pos =
            (m_vertex_attribute[v1_id].m_pos + m_vertex_attribute[v2_id].m_pos) / 2;
        p = to_double(m_vertex_attribute[v_id].m_pos);
    }

    // update face attributes
    {
        // v1 - v_new
        const auto faces1 = get_incident_fids_for_edge(v1_id, v_id);
        const simplex::Edge edge1(v1_id, v_id);
        for (const size_t fid : faces1) {
            const simplex::Face face = simplex_from_face(fid);
            const size_t opp = face.opposite_vertex(edge1).id();
            m_face_attribute[fid] = cache.faces[opp];
        }
        // v2 - v_new
        const auto faces2 = get_incident_fids_for_edge(v2_id, v_id);
        const simplex::Edge edge2(v2_id, v_id);
        for (const size_t fid : faces2) {
            const simplex::Face face = simplex_from_face(fid);
            const size_t opp = face.opposite_vertex(edge2).id();
            m_face_attribute[fid] = cache.faces[opp];
        }
        assert(faces1.size() + faces2.size() == locs.size());

        const auto [_1, eid1] = tuple_from_edge(edge1.vertices());
        const auto [_2, eid2] = tuple_from_edge(edge2.vertices());

        m_edge_attribute[eid1] = cache.old_e_attrs;
        m_edge_attribute[eid2] = cache.old_e_attrs;
        for (const auto& [vid, _] : cache.faces) {
            const auto [_tup, eid] = tuple_from_edge({{v_id, vid}});
            m_edge_attribute[eid].reset();
        }
    }

    /// update quality
    for (const Tuple& loc : locs) {
        m_face_attribute[loc.fid(*this)].m_quality = get_quality(loc);
    }

    /// update vertex attribute
    // bbox
    m_vertex_attribute[v_id].on_bbox_faces = wmtk::set_intersection(
        m_vertex_attribute[v1_id].on_bbox_faces,
        m_vertex_attribute[v2_id].on_bbox_faces);
    // surface
    m_vertex_attribute[v_id].m_is_on_surface = cache.old_e_attrs.m_is_surface_fs;

    /// update edge attribute
    for (const auto& [e, e_attr] : cache.changed_edges) {
        auto [_, eid] = tuple_from_edge(e.vertices());
        m_edge_attribute[eid] = e_attr;
    }

    m_vertex_attribute[v_id].partition_id = m_vertex_attribute[v1_id].partition_id;
    m_vertex_attribute[v_id].m_sizing_scalar =
        (m_vertex_attribute[v1_id].m_sizing_scalar + m_vertex_attribute[v2_id].m_sizing_scalar) / 2;

    return true;
}

} // namespace wmtk::components::triwild