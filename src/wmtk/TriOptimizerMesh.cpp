#include <wmtk/TriOptimizerMesh.h>

#include <wmtk/utils/AMIPS2D.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/LocalizedRetry.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/ParallelCollect.hpp>
#include <wmtk/utils/RunPass.hpp>
#include <wmtk/utils/SizingField.hpp>
#include <wmtk/utils/TupleUtils.hpp>
#include <wmtk/utils/partition_utils.hpp>

#include <igl/Timer.h>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <wmtk/utils/predicates.hpp>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

#include <queue>

namespace wmtk {

std::tuple<double, double> TriOptimizerMesh::optimization_quality_stats()
{
    return get_max_avg_energy();
}

void TriOptimizerMesh::mesh_improvement(int max_its)
{
    m_iterations_used = 0;
    if (optimization_stop_at_float() && round_and_check_all_rounded()) {
        logger().info("===== All vertices are rounded. Stop. =====");
        return;
    }

    partition_mesh_morton();
    if (optimization_bare_coarsen_passes()) {
        logger().info("========it pre========");
        local_operations({{0, 1, 0, 0}}, false);
    }

    double pre_max_metric = std::get<0>(optimization_quality_stats());
    logger().info("max energy {:.6} | stop {:.6}", pre_max_metric, optimization_stop_metric());
    int refine_cooldown = m_params.stuck_refine_cooldown;

    for (int it = 0; it < max_its; ++it) {
        m_iterations_used = it + 1;
        logger().info("\n========it {}========", it);
        optimization_iteration_begin();

        double max_metric = 0.;
        double avg_metric = 0.;
        if (!m_params.interleaved_smoothing) {
            std::tie(max_metric, avg_metric) =
                local_operations({{1, 1, 1, m_params.num_smoothing_passes}});
        } else {
            const int k = m_params.interleaved_smoothing_passes;
            const std::array<std::array<int, 4>, 3> passes = {
                {{{1, 0, 0, k}}, {{0, 1, 0, k}}, {{0, 0, 1, k}}}};
            for (const auto& ops : passes) {
                std::tie(max_metric, avg_metric) = local_operations(ops);
                if (max_metric < optimization_stop_metric()) break;
            }
        }

        logger().info("max energy {:.6} | stop {:.6}", max_metric, optimization_stop_metric());

        std::atomic<int> n_round = 0;
        std::atomic<int> n_verts = 0;
        TriMesh::for_each_vertex([&](auto& v) {
            if (m_vertex_attribute[v.vid(*this)].m_is_rounded) {
                n_round.fetch_add(1, std::memory_order_relaxed);
            }
            n_verts.fetch_add(1, std::memory_order_relaxed);
        });
        const int cnt_round = n_round.load(std::memory_order_relaxed);
        const int cnt_verts = n_verts.load(std::memory_order_relaxed);
        if (cnt_round < cnt_verts) {
            logger().info("rounded {}/{}", cnt_round, cnt_verts);
        } else {
            logger().info("All rounded!");
        }

        if (max_metric < optimization_stop_metric()) {
            if (cnt_round == cnt_verts) break;
            logger().info(
                "energy target reached, but {} of {} vertices are still un-rounded; continuing",
                cnt_verts - cnt_round,
                cnt_verts);
        }

        consolidate_mesh();
        logger().info("#V = {}, #F = {}", vert_capacity(), tri_capacity());

        if (optimization_stop_at_float() && round_and_check_all_rounded()) {
            logger().info("All vertices are rounded. Stop.");
            break;
        }

        if (refine_cooldown > 0) {
            --refine_cooldown;
        } else if (
            it > 0 && max_metric > optimization_stop_metric() &&
            optimization_stalled(pre_max_metric, max_metric)) {
            logger().info(">>>>stuck-refine (maxE {:.6} stalled)...", max_metric);
            refine_sizing_around_worst(max_metric);
            logger().info(">>>>stuck-refine finished...");
            refine_cooldown = m_params.stuck_refine_cooldown;
        }
        pre_max_metric = max_metric;
    }

    if (optimization_bare_coarsen_passes()) {
        logger().info("========it post========");
        local_operations({{0, 1, 0, 0}});
    }

    // Removing what the mesh does not need is the last thing to do, not something to
    // interleave: it trades vertices for nothing but the guarantee that the max energy does not
    // rise, which is only worth taking once the energy is where it is going to end up.
    //
    // NOT gated with the bare passes above. Those are unguarded collapse sweeps; this one runs
    // in m_coarsen_mode, which an application can use to demand more of an operation than the
    // main loop does -- topological_offset requires BOTH criteria to be inside tolerance
    // afterwards, so coarsening can only ever trade elements for a result that is still good.
    coarsen_mesh();
}

std::tuple<double, double> TriOptimizerMesh::local_operations(
    const std::array<int, 4>& ops,
    bool collapse_limit_length)
{
    igl::Timer timer;

    auto sanity_checks = [this]() {
        if (!m_params.perform_sanity_checks) return;
        logger().info("Perform sanity checks...");
        const auto edges = get_edges_by_condition([](const auto& e) { return e.m_is_surface_fs; });
        for (const auto& verts : edges) {
            if (surface_segment_is_outside(verts[0], verts[1])) {
                logger().error("Edge {} is outside!", verts);
            }
        }
        for (const Tuple& t : get_faces()) {
            if (is_inverted(t)) {
                logger().error(
                    "Face {} is inverted! Vertices = {}",
                    t.fid(*this),
                    oriented_tri_vids(t));
            }
        }
        logger().info("Sanity checks done.");
    };

    sanity_checks();
    for (int i = 0; i < int(ops.size()); ++i) {
        timer.start();
        if (i == 0) {
            for (int n = 0; n < ops[i]; ++n) {
                logger().info("==splitting {}==", n);
                split_all_edges();
                logger().info(
                    "#V = {}, #F = {} after split",
                    get_vertices().size(),
                    get_faces().size());
            }
        } else if (i == 1) {
            for (int n = 0; n < ops[i]; ++n) {
                logger().info("==collapsing {}==", n);
                collapse_all_edges(collapse_limit_length);
                logger().info(
                    "#V = {}, #F = {} after collapse",
                    get_vertices().size(),
                    get_faces().size());
            }
        } else if (i == 2) {
            for (int n = 0; n < ops[i]; ++n) {
                logger().info("==swapping {}==", n);
                if (swap_all_edges() == 0) break;
            }
        } else {
            logger().info("==smoothing ==");
            smooth_all_vertices(size_t(ops[i]));
            if (ops[i] > 0) round_all_vertices();
        }

        optimization_debug_checkpoint();
        const auto [max_metric, avg_metric] = optimization_quality_stats();
        static constexpr std::array<const char*, 4> names = {
            {"split", "collapse", "swap", "smooth"}};
        logger()
            .info("{} max energy = {:.6} avg = {:.6}", names[size_t(i)], max_metric, avg_metric);
        sanity_checks();
    }

    const auto stats = optimization_quality_stats();
    logger().info("max energy = {:.6}", std::get<0>(stats));
    logger().info("avg energy = {:.6}", std::get<1>(stats));
    logger().info("time = {:.4}s", timer.getElapsedTimeInSec());
    return stats;
}

namespace {

auto renew_swap_neighbors = [](const TriOptimizerMesh& m, auto op, auto& tris) {
    using Tuple = TriMesh::Tuple;
    std::vector<Tuple> edges;
    for (const auto& t : tris) {
        for (auto j = 0; j < 3; j++) {
            edges.push_back(m.tuple_from_edge(t.fid(m), j));
        }
    }
    unique_edge_tuples(m, edges);

    std::vector<std::pair<std::string, Tuple>> optup;
    optup.reserve(edges.size());
    for (const Tuple& e : edges) {
        optup.emplace_back(op, e);
    }
    return optup;
};

} // namespace

size_t TriOptimizerMesh::swap_all_edges()
{
    igl::Timer timer;
    timer.start();
    auto collect_all_ops = parallel_collect_edge_ops(
        *this,
        NUM_THREADS,
        [](TriOptimizerMesh&, const Tuple& e, auto& out) { out.emplace_back("edge_swap", e); });
    logger().info("#E = {}", collect_all_ops.size());
    logger().info("edge swap prepare time: {:.4}s", timer.getElapsedTimeInSec());

    size_t total_success = 0;
    run_pass(*this, PassLock::EdgeRing, "", [&](auto& executor, auto& mesh) {
        executor.renew_neighbor_tuples = renew_swap_neighbors;
        executor.priority = [](const TriOptimizerMesh& m, std::string, const Tuple& e) {
            return m.swap_weight(e);
        };
        executor.should_renew = [](auto val) { return val > 0; };
        executor.is_weight_up_to_date = [](const TriOptimizerMesh& m, auto& ele) {
            auto& [val, _, e] = ele;
            const double w = m.swap_weight(e);
            return (w > 1e-5) && ((w - val) * (w - val) < 1e-8);
        };
        total_success = run_localized_to_convergence(mesh, executor, collect_all_ops);
    });

    return total_success;
}

double TriOptimizerMesh::swap_weight(const Tuple& t) const
{
    const SmartTuple tt(*this, t);
    const auto t_opp = tt.switch_face();
    if (!t_opp || is_edge_on_surface(t)) {
        return std::numeric_limits<double>::lowest();
    }

    const size_t v0 = tt.vid();
    const size_t v1 = tt.switch_vertex().vid();
    const size_t v2 = tt.switch_edge().switch_vertex().vid();
    const size_t v3 = t_opp.value().switch_edge().switch_vertex().vid();

    const double q_before = std::max(get_quality({{v0, v1, v2}}), get_quality({{v0, v3, v1}}));
    const double q_after = std::max(get_quality({{v0, v3, v2}}), get_quality({{v2, v3, v1}}));
    return q_before - q_after;
}

bool TriOptimizerMesh::swap_edge_before(const Tuple& t)
{
    if (is_edge_on_surface(t)) {
        return false;
    }

    auto& cache = swap_cache.local();
    cache.changed_edges.clear();
    const auto incident_faces = get_incident_fids_for_edge(t);
    cache.face_tags = m_face_attribute[incident_faces[0]].tags;

    cache.max_energy = -1.;
    for (const size_t fid : incident_faces) {
        cache.max_energy = std::max(m_face_attribute[fid].m_quality, cache.max_energy);
    }

    const simplex::Edge edge = simplex_from_edge(t);
    for (const size_t fid : incident_faces) {
        for (int j = 0; j < 3; j++) {
            const Tuple tup = tuple_from_edge(fid, j);
            const simplex::Edge e = simplex_from_edge(tup);
            if (e != edge) {
                cache.changed_edges.try_emplace(e, m_edge_attribute[tup.eid(*this)]);
            }
        }
    }
    return true;
}

bool TriOptimizerMesh::swap_edge_after(const Tuple& t)
{
    auto& cache = swap_cache.local();
    const auto incident_faces = get_incident_fids_for_edge(t);

    double max_energy = -1.;
    for (const size_t fid : incident_faces) {
        if (is_inverted(fid)) {
            return false;
        }
        const double q = get_quality(fid);
        m_face_attribute[fid].m_quality = q;
        m_face_attribute[fid].tags = cache.face_tags;
        max_energy = std::max(q, max_energy);
    }
    if (max_energy >= cache.max_energy) {
        return false;
    }

    for (const auto& [e, e_attrs] : cache.changed_edges) {
        const auto [_, eid] = tuple_from_edge(e.vertices());
        m_edge_attribute[eid] = e_attrs;
    }
    m_edge_attribute[t.eid(*this)].reset();
    return true;
}

void TriOptimizerMesh::gradation_smooth_sizing(double grade, const std::vector<size_t>& seeds)
{
    utils::gradation_smooth_sizing(
        grade,
        seeds,
        [this](size_t v) -> double& { return m_vertex_attribute[v].m_sizing_scalar; },
        [this](size_t v) { return get_one_ring_vids_for_vertex_duplicate(v); });
}

void TriOptimizerMesh::partition_mesh()
{
    auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
    for (size_t i = 0; i < m_vertex_partition_id.size(); i++) {
        m_vertex_attribute[i].partition_id = m_vertex_partition_id[i];
    }
}

void TriOptimizerMesh::partition_mesh_morton()
{
    if (NUM_THREADS == 0) {
        return;
    }
    logger().info("Number of parts: {} by morton", NUM_THREADS);

    // The shared partitioner is 3D; a zero z leaves the bounding box, the scale and the
    // Morton code exactly where a 2D-specific version would put them.
    std::vector<size_t> partition_id;
    wmtk::partition_vertex_morton(
        vert_capacity(),
        [this](size_t i) {
            const Vector2d& p = m_vertex_attribute[i].m_posf;
            return Eigen::Vector3d(p[0], p[1], 0);
        },
        NUM_THREADS,
        partition_id);

    for (size_t i = 0; i < partition_id.size(); i++) {
        m_vertex_attribute[i].partition_id = partition_id[i];
    }
}

double TriOptimizerMesh::get_length2(const Tuple& l) const
{
    auto& m = *this;
    auto& v1 = l;
    auto v2 = l.switch_vertex(m);
    double length =
        (m.m_vertex_attribute[v1.vid(m)].m_posf - m.m_vertex_attribute[v2.vid(m)].m_posf)
            .squaredNorm();
    return length;
}

std::tuple<double, double> TriOptimizerMesh::get_max_avg_energy()
{
    double max_energy = -1.;
    double avg_energy = 0.;
    auto cnt = 0;

    for (int i = 0; i < tri_capacity(); i++) {
        const Tuple tup = tuple_from_tri(i);
        if (!tup.is_valid(*this)) {
            continue;
        }
        const double q = m_face_attribute[tup.fid(*this)].m_quality;
        max_energy = std::max(max_energy, q);
        avg_energy += q;
        cnt++;
    }

    avg_energy /= cnt;

    return std::make_tuple(max_energy, avg_energy);
}

bool TriOptimizerMesh::is_inverted_f(const Tuple& loc) const
{
    return is_inverted_f(loc.fid(*this));
}

bool TriOptimizerMesh::is_inverted_f(const size_t fid) const
{
    auto vs = oriented_tri_vids(fid);

    wmtk::utils::predicates::exactinit();
    auto res = wmtk::utils::predicates::orient2d(
        m_vertex_attribute[vs[0]].m_posf,
        m_vertex_attribute[vs[1]].m_posf,
        m_vertex_attribute[vs[2]].m_posf);
    if (res == wmtk::utils::predicates::Orientation::POSITIVE) {
        return false;
    }
    return true;
}

bool TriOptimizerMesh::is_inverted(const std::array<size_t, 3>& vs) const
{
    if (m_vertex_attribute[vs[0]].m_is_rounded && m_vertex_attribute[vs[1]].m_is_rounded &&
        m_vertex_attribute[vs[2]].m_is_rounded) {
        wmtk::utils::predicates::exactinit();
        auto res = wmtk::utils::predicates::orient2d(
            m_vertex_attribute[vs[0]].m_posf,
            m_vertex_attribute[vs[1]].m_posf,
            m_vertex_attribute[vs[2]].m_posf);
        if (res == wmtk::utils::predicates::Orientation::POSITIVE) {
            return false;
        }
        return true;
    } else {
        const Vector2r& v0 = m_vertex_attribute[vs[0]].m_pos;
        const Vector2r& v1 = m_vertex_attribute[vs[1]].m_pos;
        const Vector2r& v2 = m_vertex_attribute[vs[2]].m_pos;
        const Vector2r a = v1 - v0;
        const Vector2r b = v2 - v0;
        Rational res = a.x() * b.y() - a.y() * b.x();
        if (res > 0) {
            return false;
        } else {
            return true;
        }
    }
}

bool TriOptimizerMesh::is_inverted(const Tuple& loc) const
{
    auto vs = oriented_tri_vids(loc);
    return is_inverted(vs);
}

bool TriOptimizerMesh::is_inverted(const size_t fid) const
{
    auto vs = oriented_tri_vids(fid);
    return is_inverted(vs);
}

std::vector<size_t> TriOptimizerMesh::all_vertex_ids() const
{
    const std::vector<Tuple> vs = get_vertices();
    std::vector<size_t> ids;
    ids.reserve(vs.size());
    for (const Tuple& v : vs) {
        ids.push_back(v.vid(*this));
    }
    return ids;
}

bool TriOptimizerMesh::round(const Tuple& v)
{
    size_t i = v.vid(*this);
    if (m_vertex_attribute[i].m_is_rounded) {
        return true;
    }

    auto old_pos = m_vertex_attribute[i].m_pos;
    m_vertex_attribute[i].m_pos << m_vertex_attribute[i].m_posf[0], m_vertex_attribute[i].m_posf[1];
    auto conn_tets = get_one_ring_tris_for_vertex(v);
    // Set before the loop so is_inverted takes the float path: the question being asked is
    // exactly whether the ROUNDED position keeps every incident face valid.
    m_vertex_attribute[i].m_is_rounded = true;
    for (const Tuple& tet : conn_tets) {
        if (is_inverted(tet)) {
            m_vertex_attribute[i].m_is_rounded = false;
            m_vertex_attribute[i].m_pos = old_pos;
            return false;
        }
    }

    return true;
}

double TriOptimizerMesh::get_quality(const std::array<size_t, 3>& vs) const
{
    std::array<Vector2d, 3> ps;
    for (size_t k = 0; k < 3; k++) {
        ps[k] = m_vertex_attribute[vs[k]].m_posf;
    }
    double energy = -1.;
    {
        std::array<double, 6> T;
        for (size_t k = 0; k < 3; k++)
            for (size_t j = 0; j < 2; j++) {
                T[k * 2 + j] = ps[k][j];
            }
        energy = AMIPS2D_energy(T);
    }
    if (std::isinf(energy) || std::isnan(energy) || energy < 2 - 1e-3) {
        return MAX_ENERGY;
    }
    return energy;
}

double TriOptimizerMesh::get_quality(const Tuple& loc) const
{
    auto its = oriented_tri_vids(loc);
    return get_quality(its);
}

double TriOptimizerMesh::get_quality(const size_t fid) const
{
    auto its = oriented_tri_vids(fid);
    return get_quality(its);
}

std::vector<std::array<size_t, 2>> TriOptimizerMesh::get_edges_by_condition(
    std::function<bool(const EdgeAttributes&)> cond) const
{
    std::vector<std::array<size_t, 2>> res;
    for (const Tuple& e : get_edges()) {
        size_t eid = e.eid(*this);
        if (cond(m_edge_attribute[eid])) {
            res.push_back({{e.vid(*this), e.switch_vertex(*this).vid(*this)}});
        }
    }
    return res;
}

bool TriOptimizerMesh::is_edge_on_surface(const Tuple& loc) const
{
    const auto vs = get_edge_vids(loc);
    if (!m_vertex_attribute.at(vs[0]).m_is_on_surface ||
        !m_vertex_attribute.at(vs[1]).m_is_on_surface) {
        return false;
    }

    const size_t eid = loc.eid(*this);
    return m_edge_attribute[eid].m_is_surface_fs;
}
bool TriOptimizerMesh::is_edge_on_surface(const std::array<size_t, 2>& vids) const
{
    if (!m_vertex_attribute.at(vids[0]).m_is_on_surface ||
        !m_vertex_attribute.at(vids[1]).m_is_on_surface) {
        return false;
    }

    const auto [_, eid] = tuple_from_edge(vids);
    return m_edge_attribute[eid].m_is_surface_fs;
}
bool TriOptimizerMesh::is_edge_on_bbox(const Tuple& loc) const
{
    const auto vs = get_edge_vids(loc);
    if (m_vertex_attribute.at(vs[0]).on_bbox_faces.empty() ||
        m_vertex_attribute.at(vs[1]).on_bbox_faces.empty()) {
        return false;
    }

    const size_t eid = loc.eid(*this);
    return m_edge_attribute[eid].m_is_bbox_fs >= 0;
}

bool TriOptimizerMesh::is_edge_on_bbox(const std::array<size_t, 2>& vids) const
{
    if (m_vertex_attribute.at(vids[0]).on_bbox_faces.empty() ||
        m_vertex_attribute.at(vids[1]).on_bbox_faces.empty()) {
        return false;
    }
    const auto [_, eid] = tuple_from_edge(vids);
    return m_edge_attribute[eid].m_is_bbox_fs >= 0;
}

} // namespace wmtk
