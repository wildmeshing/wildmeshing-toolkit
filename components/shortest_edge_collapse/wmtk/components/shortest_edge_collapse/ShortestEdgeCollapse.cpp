#include "ShortestEdgeCollapse.h"
#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/TupleUtils.hpp>

#include <chrono>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <paraviewo/VTUWriter.hpp>

namespace wmtk::components::shortest_edge_collapse {

ShortestEdgeCollapse::ShortestEdgeCollapse(
    std::vector<Eigen::Vector3d> _m_vertex_positions,
    int num_threads,
    bool use_exact_envelope)
{
    NUM_THREADS = (num_threads);
    m_envelope.use_exact = use_exact_envelope;
    p_vertex_attrs = &vertex_attrs;

    vertex_attrs.resize(_m_vertex_positions.size());

    for (auto i = 0; i < _m_vertex_positions.size(); i++)
        vertex_attrs[i] = {_m_vertex_positions[i], 0, false};
}

void ShortestEdgeCollapse::freeze_boundary()
{
    for (const Tuple& e : get_edges()) {
        if (is_boundary_edge(e)) {
            vertex_attrs[e.vid(*this)].freeze = true;
            vertex_attrs[e.switch_vertex(*this).vid(*this)].freeze = true;
        }
    }
}

void ShortestEdgeCollapse::create_mesh(
    size_t n_vertices,
    const std::vector<std::array<size_t, 3>>& tris,
    const std::vector<size_t>& frozen_verts,
    double eps)
{
    wmtk::TriMesh::init(n_vertices, tris);

    if (eps > 0) {
        std::vector<Eigen::Vector3d> V(n_vertices);
        std::vector<Eigen::Vector3i> F(tris.size());
        for (size_t i = 0; i < V.size(); i++) {
            V[i] = vertex_attrs[i].pos;
        }
        for (size_t i = 0; i < F.size(); ++i) {
            F[i] << (int)tris[i][0], (int)tris[i][1], (int)tris[i][2];
        }
        m_envelope.init(V, F, eps);
        m_has_envelope = true;
    }
    partition_mesh();
    for (size_t v : frozen_verts) {
        vertex_attrs[v].freeze = true;
    }
    freeze_boundary();
}

void ShortestEdgeCollapse::partition_mesh()
{
    auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
    for (auto i = 0; i < m_vertex_partition_id.size(); i++)
        vertex_attrs[i].partition_id = m_vertex_partition_id[i];
}


bool ShortestEdgeCollapse::invariants(const std::vector<Tuple>& new_tris)
{
    if (m_has_envelope) {
        for (auto& t : new_tris) {
            std::array<Eigen::Vector3d, 3> tris;
            auto vs = oriented_tri_vertices(t);
            for (auto j = 0; j < 3; j++) tris[j] = vertex_attrs[vs[j].vid(*this)].pos;
            bool outside = m_envelope.is_outside(tris);
            if (outside) return false;
        }
    }
    return true;
}

bool ShortestEdgeCollapse::write_triangle_mesh(std::string path)
{
    Eigen::MatrixXd V = Eigen::MatrixXd::Zero(vert_capacity(), 3);
    for (auto& t : get_vertices()) {
        auto i = t.vid(*this);
        V.row(i) = vertex_attrs[i].pos;
    }

    Eigen::MatrixXi F = Eigen::MatrixXi::Constant(tri_capacity(), 3, -1);
    for (auto& t : get_faces()) {
        auto i = t.fid(*this);
        auto vs = oriented_tri_vertices(t);
        for (int j = 0; j < 3; j++) {
            F(i, j) = (int)vs[j].vid(*this);
        }
    }

    logger().info("Write {}", path);
    return igl::write_triangle_mesh(path, V, F);
}

void ShortestEdgeCollapse::write_vtu(const std::string& path)
{
    const std::string out_path = path + ".vtu";
    logger().info("Write {}", out_path);

    MatrixXd V = MatrixXd::Zero(vert_capacity(), 3);
    MatrixXi F = MatrixXi::Zero(tri_capacity(), 3);

    VectorXd freeze(vertex_attrs.size());
    freeze.setZero();

    for (Tuple& t : get_vertices()) {
        const size_t i = t.vid(*this);
        V.row(i) = vertex_attrs[i].pos;
        freeze(i) = vertex_attrs[i].freeze ? 1 : 0;
    }

    for (Tuple& t : get_faces()) {
        const size_t i = t.fid(*this);
        const auto vs = oriented_tri_vertices(t);
        for (int j = 0; j < 3; j++) {
            F(i, j) = (int)vs[j].vid(*this);
        }
    }

    paraviewo::VTUWriter writer;
    writer.add_field("freeze", freeze);
    writer.write_mesh(out_path, V, F, paraviewo::CellType::Triangle);
}

bool ShortestEdgeCollapse::collapse_edge_before(const Tuple& t)
{
    if (!TriMesh::collapse_edge_before(t)) return false;

    // v1 is removed by the collapse, v2 survives (TriMesh::collapse_edge_conn keeps vid2).
    const size_t v1 = t.vid(*this);
    const size_t v2 = t.switch_vertex(*this).vid(*this);
    auto& cache = position_cache.local();
    cache.v1_frozen = vertex_attrs[v1].freeze;
    cache.v2_frozen = vertex_attrs[v2].freeze;

    // Two frozen endpoints cannot be merged without moving one of them. In particular this
    // is what keeps the outline of an open surface from retracting: the envelope is a
    // containment test, so it would not notice a boundary sliding inwards along itself.
    if (cache.v1_frozen && cache.v2_frozen) {
        return false;
    }

    cache.v1p = vertex_attrs[v1].pos;
    cache.v2p = vertex_attrs[v2].pos;
    return true;
}


bool ShortestEdgeCollapse::collapse_edge_after(const TriMesh::Tuple& t)
{
    const auto& cache = position_cache.local();
    // Collapse onto the frozen endpoint when exactly one is frozen, so its position is
    // preserved exactly and the collapse is still allowed; onto the midpoint otherwise.
    // Rejecting these outright, as this used to, froze not just the boundary but every
    // vertex adjacent to it.
    const Eigen::Vector3d p = cache.v1_frozen   ? cache.v1p
                              : cache.v2_frozen ? cache.v2p
                                                : (cache.v1p + cache.v2p) / 2.0;
    const size_t vid = t.vid(*this);
    vertex_attrs[vid].pos = p;
    // The survivor now stands exactly where the frozen vertex stood, so it takes over its
    // frozen role -- otherwise a later collapse could move that position after all.
    vertex_attrs[vid].freeze = cache.v1_frozen || cache.v2_frozen;

    return true;
}


std::vector<TriMesh::Tuple> ShortestEdgeCollapse::new_edges_after(
    const std::vector<TriMesh::Tuple>& tris) const
{
    std::vector<TriMesh::Tuple> new_edges;
    std::vector<size_t> one_ring_fid;

    for (auto t : tris) {
        for (auto j = 0; j < 3; j++) {
            new_edges.push_back(tuple_from_edge(t.fid(*this), j));
        }
    }
    wmtk::unique_edge_tuples(*this, new_edges);
    return new_edges;
}

bool ShortestEdgeCollapse::collapse_shortest(int target_vert_number)
{
    size_t initial_size = get_vertices().size();
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    for (auto& loc : get_edges()) collect_all_ops.emplace_back("edge_collapse", loc);

    // Progress reporting.
    //
    // collapse_shortest is one call that can run for many minutes with nothing to show for
    // it: with an exact envelope essentially all the time goes into the envelope predicate,
    // and from outside that is indistinguishable from a hang. The renew hook fires once per
    // successful collapse, which is the cheapest place to count from without touching the
    // scheduler.
    //
    // Timed rather than counted, because the two are not interchangeable here: simplifying
    // the crown takes 68k collapses with the link condition off and 64k with it on, but the
    // wall clock differs by orders of magnitude depending on whether an envelope is
    // configured. Any fixed count is silent on one run and chatty on another; a time budget
    // is quiet for passes that finish quickly and talks steadily for ones that do not.
    std::atomic<size_t> n_collapsed{0};
    std::atomic<long long> last_report_ms{0};
    const auto started = std::chrono::steady_clock::now();
    constexpr long long REPORT_EVERY_MS = 10000;
    constexpr size_t CLOCK_CHECK_STRIDE = 1024; // reading the clock per collapse is wasteful

    auto renew = [&](auto& m, auto op, auto& tris) {
        const size_t done = ++n_collapsed;
        if (done % CLOCK_CHECK_STRIDE == 0) {
            const long long ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                     std::chrono::steady_clock::now() - started)
                                     .count();
            long long last = last_report_ms.load(std::memory_order_relaxed);
            // one thread wins the slot, the rest carry on
            if (ms - last >= REPORT_EVERY_MS && last_report_ms.compare_exchange_strong(last, ms)) {
                const double secs = double(ms) / 1000.0;
                logger().info(
                    "\tcollapsed {} edges, ~{} vertices left, {:.0f}/s, {:.0f}s",
                    done,
                    initial_size > done ? initial_size - done : 0,
                    secs > 0 ? done / secs : 0.0,
                    secs);
            }
        }
        auto edges = m.new_edges_after(tris);
        auto optup = std::vector<std::pair<std::string, Tuple>>();
        for (auto& e : edges) optup.emplace_back("edge_collapse", e);
        return optup;
    };
    auto measure_len2 = [](auto& m, auto op, const Tuple& new_e) {
        auto len2 =
            (m.vertex_attrs[new_e.vid(m)].pos - m.vertex_attrs[new_e.switch_vertex(m).vid(m)].pos)
                .squaredNorm();
        return -len2;
    };
    auto setup_and_execute = [&](auto& executor) {
        executor.num_threads = NUM_THREADS;
        executor.renew_neighbor_tuples = renew;
        executor.priority = measure_len2;
        executor.stopping_criterion_checking_frequency =
            target_vert_number > 0 ? (initial_size - target_vert_number - 1)
                                   : std::numeric_limits<int>::max();
        executor.stopping_criterion = [](auto& m) { return true; };
        executor(*this, collect_all_ops);
    };

    if (NUM_THREADS > 0) {
        auto executor = wmtk::ExecutePass<ShortestEdgeCollapse>(ExecutionPolicy::kPartition);
        executor.lock_vertices = [](auto& m, const auto& e, int task_id) {
            return m.try_set_edge_mutex_two_ring(e, task_id);
        };
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<ShortestEdgeCollapse>(ExecutionPolicy::kSeq);
        setup_and_execute(executor);
    }
    return true;
}

} // namespace wmtk::components::shortest_edge_collapse