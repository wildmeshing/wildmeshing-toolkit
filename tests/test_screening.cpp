#include <wmtk/TriMesh.h>
#include <wmtk/AttributeCollection.hpp>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/OperationDryRun.hpp>
#include <wmtk/threading/enumerable_thread_specific.hpp>
#include <wmtk/utils/Logger.hpp>

#include <spdlog/sinks/ostream_sink.h>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <memory>
#include <random>
#include <sstream>
#include <stdexcept>
#include <vector>

using namespace wmtk;

namespace {

// ---------------------------------------------------------------------------------------------
// Dry runs on a TriMesh. See the TetMesh twin in test_operations.cpp.
// ---------------------------------------------------------------------------------------------

/// Every face, every vertex's star, and the slot counts.
std::vector<std::vector<size_t>> tri_mesh_state(const TriMesh& m)
{
    std::vector<std::vector<size_t>> s;
    for (const auto& f : m.get_faces()) {
        const auto v = m.oriented_tri_vids(f);
        s.push_back({f.fid(m), v[0], v[1], v[2]});
    }
    for (const auto& v : m.get_vertices()) {
        std::vector<size_t> star;
        for (const auto& f : m.get_one_ring_tris_for_vertex(v)) star.push_back(f.fid(m));
        std::sort(star.begin(), star.end());
        star.insert(star.begin(), v.vid(m));
        s.push_back(star);
    }
    s.push_back({m.tri_capacity(), m.vert_capacity()});
    return s;
}

/// The triangles of an n x n grid of vertices, two per square.
std::vector<std::array<size_t, 3>> grid_tris(const size_t n)
{
    const auto vid = [n](size_t i, size_t j) { return j * n + i; };
    std::vector<std::array<size_t, 3>> tris;
    for (size_t j = 0; j + 1 < n; ++j) {
        for (size_t i = 0; i + 1 < n; ++i) {
            tris.push_back({{vid(i, j), vid(i + 1, j), vid(i, j + 1)}});
            tris.push_back({{vid(i + 1, j), vid(i + 1, j + 1), vid(i, j + 1)}});
        }
    }
    return tris;
}

/// As check_dry_runs in test_operations.cpp: no change, and no refusal of what would succeed.
template <class Simplices, class Op>
int check_tri_dry_runs(const Simplices& simplices, const Op& op, bool exact)
{
    const auto make = [] {
        auto m = std::make_unique<TriMesh>();
        m->init(16, grid_tris(4));
        return m;
    };
    const size_t n = simplices(*make()).size();
    REQUIRE(n > 0);
    int passed = 0;
    for (size_t i = 0; i < n; ++i) {
        auto dry = make();
        const auto before = tri_mesh_state(*dry);
        bool d = false;
        {
            OperationDryRunScope scope;
            d = op(*dry, simplices(*dry)[i]);
        }
        CHECK(tri_mesh_state(*dry) == before);
        // As the scheduler does before every serial operation: storage for the worst case.
        auto real = make();
        real->reserve_free_slots(real->cell_slot_bound(simplices(*real)[i]), 1);
        const bool r = op(*real, simplices(*real)[i]);
        if (r) CHECK(d);
        if (exact) CHECK(d == r);
        passed += d ? 1 : 0;
    }
    return passed;
}

// ---------------------------------------------------------------------------------------------
// A screened pass: splitting every edge longer than a threshold, longest first, on a jittered
// grid. The split order is the one the optimizers' split passes use: an edge waits (must_wait)
// while a strictly longer edge of an incident triangle, itself over the threshold, is queued.
// ---------------------------------------------------------------------------------------------

struct Position
{
    Vector2d p;
};

class SplitMesh : public TriMesh
{
public:
    AttributeCollection<Position> m_vertex_attribute;
    double threshold2 = 0;
    /// After-hook calls; the after-hook throws on call number `throw_at` (0: never).
    std::atomic<int> after_calls{0};
    int throw_at = 0;

    SplitMesh() { p_vertex_attrs = &m_vertex_attribute; }

    void build(const size_t n, const unsigned seed)
    {
        init(n * n, grid_tris(n));
        m_vertex_attribute.resize(n * n);
        std::mt19937 rng(seed);
        std::uniform_real_distribution<double> jitter(-0.3, 0.3);
        for (size_t j = 0; j < n; ++j) {
            for (size_t i = 0; i < n; ++i) {
                m_vertex_attribute[j * n + i].p =
                    Vector2d(double(i) + jitter(rng), double(j) + jitter(rng));
            }
        }
    }

    /// Partitions per thread: 1, as the optimizers partition their meshes, or more, as the
    /// input simplification does (ShortestEdgeCollapse::partition_mesh).
    size_t parts_per_thread = 1;
    size_t get_partition_id(const Tuple& t) const
    {
        return t.vid(*this) % (size_t(std::max(1, NUM_THREADS)) * parts_per_thread);
    }

    double len2(const Tuple& e) const
    {
        const auto& a = m_vertex_attribute.at(e.vid(*this)).p;
        const auto& b = m_vertex_attribute.at(e.switch_vertex(*this).vid(*this)).p;
        return (a - b).squaredNorm();
    }

    /// The other two edges of each triangle incident to `e`.
    std::vector<Tuple> neighbour_edges(const Tuple& e) const
    {
        std::vector<Tuple> out;
        std::vector<Tuple> faces{e};
        if (const auto f = e.switch_face(*this)) faces.push_back(*f);
        for (const Tuple& f : faces) {
            out.push_back(f.switch_edge(*this));
            out.push_back(f.switch_vertex(*this).switch_edge(*this));
        }
        return out;
    }

    struct Cache
    {
        Vector2d a, b;
    };
    wmtk::threading::enumerable_thread_specific<Cache> cache;

    bool split_edge_before(const Tuple& t) override
    {
        auto& c = cache.local();
        c.a = m_vertex_attribute.at(t.vid(*this)).p;
        c.b = m_vertex_attribute.at(t.switch_vertex(*this).vid(*this)).p;
        return true;
    }
    bool split_edge_after(const Tuple& t) override
    {
        if (++after_calls == throw_at) throw std::runtime_error("after-hook failure");
        const auto& c = cache.local();
        m_vertex_attribute[t.switch_vertex(*this).vid(*this)].p = (c.a + c.b) / 2;
        return true;
    }
};

using Executor = ExecutePass<SplitMesh>;

/// An executor for the split pass; `driver_sentinel` is what its renewal returns when asked to
/// renew nothing, so a test can tell the driver's function from the screened call's wrapper.
std::unique_ptr<Executor> split_executor(int threads)
{
    auto ex = std::make_unique<Executor>(ExecutionPolicy::kPartition);
    ex->num_threads = threads;
    ex->lock_vertices = [](SplitMesh& m, const SplitMesh::Tuple& e, int task_id) {
        return m.try_set_edge_mutex_two_ring(e, task_id);
    };
    ex->priority = [](const SplitMesh& m, const Op&, const SplitMesh::Tuple& e) {
        return m.len2(e);
    };
    ex->is_weight_up_to_date = [](const SplitMesh& m,
                                  const std::tuple<double, Op, SplitMesh::Tuple>& w) {
        const auto& [weight, op, e] = w;
        return weight == m.len2(e) && weight > m.threshold2;
    };
    ex->renew_neighbor_tuples =
        [](const SplitMesh& m, Op, const std::vector<SplitMesh::Tuple>& ts) {
            std::vector<std::pair<Op, SplitMesh::Tuple>> out;
            if (ts.empty()) {
                out.emplace_back("driver_sentinel", SplitMesh::Tuple());
                return out;
            }
            for (const auto& f : ts) {
                out.emplace_back("edge_split", f);
                out.emplace_back("edge_split", f.switch_edge(m));
                out.emplace_back("edge_split", f.switch_vertex(m).switch_edge(m));
            }
            return out;
        };
    Executor* raw = ex.get();
    ex->queue_key = [](const SplitMesh& m, const Op&, const SplitMesh::Tuple& e) {
        return edge_queue_key(e.vid(m), e.switch_vertex(m).vid(m));
    };
    ex->must_wait = [raw](const SplitMesh& m, const Op&, const SplitMesh::Tuple& e) {
        const double l2 = m.len2(e);
        for (const auto& f : m.neighbour_edges(e)) {
            const double f2 = m.len2(f);
            if (f2 > l2 && f2 > m.threshold2 &&
                raw->is_queued(edge_queue_key(f.vid(m), f.switch_vertex(m).vid(m)))) {
                return true;
            }
        }
        return false;
    };
    return ex;
}

std::vector<std::pair<Op, SplitMesh::Tuple>> all_edges(const SplitMesh& m)
{
    std::vector<std::pair<Op, SplitMesh::Tuple>> ops;
    for (const auto& e : m.get_edges()) ops.emplace_back("edge_split", e);
    return ops;
}

/// Captures what the wmtk logger prints while it lives.
struct LogCapture
{
    std::ostringstream text;
    std::shared_ptr<spdlog::sinks::ostream_sink_mt> sink =
        std::make_shared<spdlog::sinks::ostream_sink_mt>(text);
    LogCapture() { logger().sinks().push_back(sink); }
    ~LogCapture()
    {
        auto& sinks = logger().sinks();
        sinks.erase(std::remove(sinks.begin(), sinks.end(), sink), sinks.end());
    }
};

} // namespace

TEST_CASE(
    "TriMesh dry runs change nothing and refuse only what the operation refuses",
    "[operations]")
{
    using Tuple = TriMesh::Tuple;
    const auto edges = [](const TriMesh& m) { return m.get_edges(); };
    const auto faces = [](const TriMesh& m) { return m.get_faces(); };
    const auto vertices = [](const TriMesh& m) { return m.get_vertices(); };
    // A fresh output vector per call, as the scheduler passes: the operations append to it.
    const auto split_edge = [](TriMesh& m, const Tuple& t) {
        std::vector<Tuple> out;
        return m.split_edge(t, out);
    };
    const auto collapse_edge = [](TriMesh& m, const Tuple& t) {
        std::vector<Tuple> out;
        return m.collapse_edge(t, out);
    };
    const auto swap_edge = [](TriMesh& m, const Tuple& t) {
        std::vector<Tuple> out;
        return m.swap_edge(t, out);
    };
    const auto split_face = [](TriMesh& m, const Tuple& t) {
        std::vector<Tuple> out;
        return m.split_face(t, out);
    };
    const auto smooth = [](TriMesh& m, const Tuple& t) { return m.smooth_vertex(t); };

    const int passed_1 = check_tri_dry_runs(edges, split_edge, true);
    CHECK(passed_1 > 0);
    const int passed_2 = check_tri_dry_runs(edges, collapse_edge, false);
    CHECK(passed_2 > 0);
    const int passed_3 = check_tri_dry_runs(edges, swap_edge, false);
    CHECK(passed_3 > 0);
    const int passed_4 = check_tri_dry_runs(faces, split_face, true);
    CHECK(passed_4 > 0);
    const int passed_5 = check_tri_dry_runs(vertices, smooth, true);
    CHECK(passed_5 > 0);
}

TEST_CASE("a screened split pass splits everything, in order, and keeps its books", "[screening]")
{
    // 40 x 40: the first rounds commit through the parallel queues, the tail of the pass round by
    // round inline. 6 x 6: inline only (at most 64 survivors per round).
    for (const size_t n : {size_t(40), size_t(6)}) {
        for (const int threads : {1, 3, 8}) {
            SplitMesh m;
            m.build(n, 7);
            m.threshold2 = 0.6 * 0.6;
            m.NUM_THREADS = threads;
            auto ex = split_executor(threads);
            const size_t verts_before = m.get_vertices().size();
            LogCapture log;
            REQUIRE((*ex)(m, all_edges(m)));

            // Every split succeeds and adds one vertex.
            CHECK(size_t(ex->get_cnt_success()) == m.get_vertices().size() - verts_before);
            CHECK(ex->get_cnt_success() > 0);
            // Nothing over the threshold is left: waiting operations were carried to the next
            // round, not lost.
            for (const auto& e : m.get_edges()) CHECK(m.len2(e) <= m.threshold2);
            CHECK(m.check_mesh_connectivity_validity());
            // The order holds without a defect, and the queue_key counts balance.
            CHECK(ex->wait_defects() == 0);
            CHECK(log.text.str().find("queue_key bookkeeping") == std::string::npos);
            CHECK(log.text.str().find("screened:") != std::string::npos);
            // The driver's renewal is back in place.
            CHECK(ex->renew_neighbor_tuples(m, "edge_split", {}).size() == 1);
        }
    }
}

TEST_CASE("a screened pass restores its hooks when an operation throws", "[screening]")
{
    for (const int throw_at : {1, 40, 400}) {
        SplitMesh m;
        m.build(30, 3);
        m.threshold2 = 0.6 * 0.6;
        m.throw_at = throw_at;
        m.NUM_THREADS = 4;
        auto ex = split_executor(4);
        CHECK_THROWS_AS((*ex)(m, all_edges(m)), std::runtime_error);
        CHECK(ex->renew_neighbor_tuples(m, "edge_split", {}).size() == 1);
    }
}

TEST_CASE("a screened pass ends when its operations can only wait", "[screening]")
{
    // A must_wait that never lets anything go: the unscreened scheduler ends such a pass through
    // its serial queue, failing every operation as a defect. A screened pass must end too.
    SplitMesh m;
    m.build(12, 5);
    m.threshold2 = 0.6 * 0.6;
    m.NUM_THREADS = 4;
    auto ex = split_executor(4);
    ex->must_wait = [](const SplitMesh&, const Op&, const SplitMesh::Tuple&) { return true; };
    const auto ops = all_edges(m);
    size_t due = 0;
    for (const auto& [op, e] : ops) due += m.len2(e) > m.threshold2 ? 1 : 0;
    REQUIRE(due > 64); // enough for a parallel commit
    REQUIRE((*ex)(m, ops));
    CHECK(ex->get_cnt_success() == 0);
    CHECK(size_t(ex->get_cnt_fail()) == due);
    CHECK(ex->wait_defects() == due);
}

TEST_CASE("a pass over more partitions than threads splits everything", "[screening]")
{
    // Tasks take partitions one after another, and carry the operations a lost lock race set
    // aside into the next one they take. Partitions by vid modulo their count interleave
    // everywhere, so nearly every ring crosses partitions and races are lost all the time.
    // Nothing set aside may be lost on the way, screened or not.
    for (const bool screened : {true, false}) {
        for (const int threads : {2, 8}) {
            SplitMesh m;
            m.build(40, 11);
            m.threshold2 = 0.6 * 0.6;
            m.NUM_THREADS = threads;
            m.parts_per_thread = 4;
            auto ex = split_executor(threads);
            ex->screen_before_commit = screened;
            const size_t verts_before = m.get_vertices().size();
            LogCapture log;
            REQUIRE((*ex)(m, all_edges(m)));

            CHECK(size_t(ex->get_cnt_success()) == m.get_vertices().size() - verts_before);
            CHECK(ex->get_cnt_success() > 0);
            for (const auto& e : m.get_edges()) CHECK(m.len2(e) <= m.threshold2);
            CHECK(m.check_mesh_connectivity_validity());
            CHECK(ex->wait_defects() == 0);
            CHECK(log.text.str().find("queue_key bookkeeping") == std::string::npos);
        }
    }
}
