#include "ShortestEdgeCollapse.h"
#include <wmtk/TriMesh.h>
#include <wmtk/utils/VectorUtils.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/TupleUtils.hpp>

#include <algorithm>
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
    // A large envelope check may take the threads that have run out of work; see
    // SampleEnvelope::set_max_threads.
    m_envelope.set_max_threads(NUM_THREADS);
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
    double eps,
    double boundary_eps)
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
    if (boundary_eps > 0) {
        // The edge envelope builds its exact structure only if use_exact is set now, so take
        // the surface envelope's choice; collapse_shortest() re-syncs the flag in case the
        // caller flips m_envelope.use_exact afterwards, as tetwild does.
        m_boundary_envelope.init(*this, vertex_attrs, boundary_eps, m_envelope.use_exact);
    } else {
        freeze_boundary();
    }
}

void ShortestEdgeCollapse::partition_mesh()
{
    // Several partitions per thread, which the scheduler's tasks take one after another until
    // none is left. Partitions with the same number of vertices can hold very different amounts
    // of work -- a flat region coarsening into large triangles costs far more envelope checks
    // than a curved one -- and with one per thread, the run waited on the heaviest. On tetwild's
    // simplification of a set of EMI cell surfaces at 16 threads, 4 per thread was best (5.4,
    // 4.5, 5.1, 5.8 s for 1, 4, 8, 16); more cuts more rings across partitions and loses more
    // lock races.
    constexpr int kPartitionsPerThread = 4;
    auto m_vertex_partition_id = partition_TriMesh(
        *this,
        NUM_THREADS > 1 ? NUM_THREADS * kPartitionsPerThread : NUM_THREADS);
    for (auto i = 0; i < m_vertex_partition_id.size(); i++)
        vertex_attrs[i].partition_id = m_vertex_partition_id[i];
}


bool ShortestEdgeCollapse::invariants(const std::vector<Tuple>& new_tris)
{
    // First: it touches only boundary edges, and is far cheaper than the surface test.
    if (!m_boundary_envelope.boundary_edges_inside(*this, vertex_attrs, new_tris)) {
        return false;
    }
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
    // TriMesh::collapse_edge_before is the classical link condition, applied only when
    // set_use_link_condition() left it on. tetwild sets it from simplify_use_link_condition,
    // off by default, and the simplification then may change the surface topology, including
    // creating non-manifold edges and vertices.
    if (!TriMesh::collapse_edge_before(t)) return false;

    // v1 is removed by the collapse, v2 survives (TriMesh::collapse_edge_conn keeps vid2).
    const size_t v1 = t.vid(*this);
    const size_t v2 = t.switch_vertex(*this).vid(*this);
    auto& cache = position_cache.local();
    cache.v1_frozen = vertex_attrs[v1].freeze;
    cache.v2_frozen = vertex_attrs[v2].freeze;

    // Two frozen endpoints cannot be merged without moving one of them. With a frozen boundary
    // this is what keeps the outline of an open surface from retracting: the surface envelope
    // is a containment test, so it would not notice a boundary sliding inwards along the
    // surface. With a boundary envelope that job is m_boundary_envelope's instead.
    if (cache.v1_frozen && cache.v2_frozen) {
        return false;
    }

    // Read on the connectivity before the collapse, for the placement in collapse_edge_after.
    // Only input_boundary vertices count, so that boundary torn open elsewhere is placed as it
    // always was, and so that only they pay for the ring walk.
    cache.v1_input_boundary = vertex_attrs[v1].input_boundary;
    cache.v2_input_boundary = vertex_attrs[v2].input_boundary;
    cache.v1_on_boundary = wmtk::BoundaryEnvelope::on_input_boundary(*this, vertex_attrs, t);
    cache.v2_on_boundary =
        wmtk::BoundaryEnvelope::on_input_boundary(*this, vertex_attrs, t.switch_vertex(*this));

    cache.v1p = vertex_attrs[v1].pos;
    cache.v2p = vertex_attrs[v2].pos;

    cache.ring_len2.clear();
    if (max_edge_length > 0) {
        for (const size_t v : {v1, v2}) {
            get_one_ring_vids_for_vertex_duplicate(v, cache.one_ring);
            for (const size_t w : cache.one_ring) {
                if (w == v1 || w == v2) continue;
                const double l2 = (vertex_attrs[w].pos - vertex_attrs[v].pos).squaredNorm();
                auto it = std::find_if(
                    cache.ring_len2.begin(),
                    cache.ring_len2.end(),
                    [w](const auto& r) { return r.first == w; });
                if (it == cache.ring_len2.end()) {
                    cache.ring_len2.emplace_back(w, l2);
                } else {
                    it->second = std::max(it->second, l2);
                }
            }
        }
    }
    return true;
}


bool ShortestEdgeCollapse::collapse_edge_after(const TriMesh::Tuple& t)
{
    const auto& cache = position_cache.local();
    // Collapse onto the frozen endpoint when exactly one is frozen, so its position is
    // preserved exactly and the collapse is still allowed; onto the midpoint otherwise.
    // Rejecting these outright, as this used to, froze not just the boundary but every
    // vertex adjacent to it.
    //
    // A boundary endpoint is preferred the same way when the other one is interior: the
    // midpoint would pull the outline into the surface, which the boundary envelope then
    // refuses, so the collapse would be lost rather than taken. Two boundary endpoints get the
    // midpoint -- on the boundary when the edge is a boundary edge, and judged by the envelope
    // when it is not.
    const Eigen::Vector3d p = cache.v1_frozen                                 ? cache.v1p
                              : cache.v2_frozen                               ? cache.v2p
                              : cache.v1_on_boundary && !cache.v2_on_boundary ? cache.v1p
                              : cache.v2_on_boundary && !cache.v1_on_boundary
                                  ? cache.v2p
                                  : (cache.v1p + cache.v2p) / 2.0;

    // See max_edge_length. Checked here, before the envelope, which is far more expensive.
    if (max_edge_length > 0) {
        const double max2 = max_edge_length * max_edge_length;
        for (const auto& [w, before2] : cache.ring_len2) {
            const double after2 = (p - vertex_attrs[w].pos).squaredNorm();
            if (after2 > max2 && before2 <= max2) return false;
        }
    }

    const size_t vid = t.vid(*this);
    vertex_attrs[vid].pos = p;
    // The survivor now stands exactly where the frozen vertex stood, so it takes over its
    // frozen role -- otherwise a later collapse could move that position after all.
    vertex_attrs[vid].freeze = cache.v1_frozen || cache.v2_frozen;
    vertex_attrs[vid].input_boundary = cache.v1_input_boundary || cache.v2_input_boundary;

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
    // Answer boundary queries with the same predicate as surface ones. Callers choose it by
    // setting m_envelope.use_exact between create_mesh() and here; the exact structure of
    // the boundary envelope exists only if the flag was on when it was built.
    m_boundary_envelope.set_use_exact(m_envelope.use_exact);

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