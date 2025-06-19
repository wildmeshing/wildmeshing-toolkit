#include "SimplicialEmbeddingTriMesh.hpp"

#include <paraviewo/VTUWriter.hpp>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/io/TriVTUWriter.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/TupleUtils.hpp>

namespace wmtk::components::simplicial_embedding {

struct
{
    bool operator()(SimplicialEmbeddingTriMesh& m, const TriMesh::Tuple& e, int task_id)
    {
        // TODO: this should not be here
        return m.try_set_edge_mutex_two_ring(e, task_id);
    }
} edge_locker;

SimplicialEmbeddingTriMesh::SimplicialEmbeddingTriMesh()
{
    p_vertex_attrs = &vertex_attrs;
    p_edge_attrs = &edge_attrs;
    p_face_attrs = &face_attrs;
}

void SimplicialEmbeddingTriMesh::set_positions(const std::vector<Vector2d>& vertex_positions)
{
    assert(vertex_positions.size() == vert_capacity());
    vertex_attrs.resize(vertex_positions.size());

    for (int64_t i = 0; i < vertex_positions.size(); i++) {
        vertex_attrs[i].pos = vertex_positions[i];
    }
}

void SimplicialEmbeddingTriMesh::set_positions(const MatrixXd& V)
{
    assert(V.cols() == 2);
    std::vector<Vector2d> vertex_positions;
    vertex_positions.resize(V.rows());

    for (int i = 0; i < V.rows(); ++i) {
        for (int j = 0; j < 2; ++j) {
            vertex_positions[i][j] = V(i, j);
        }
    }

    set_positions(vertex_positions);
}

void SimplicialEmbeddingTriMesh::set_num_threads(const int64_t num_threads)
{
    NUM_THREADS = num_threads;
}

void SimplicialEmbeddingTriMesh::set_vertex_tag(const Tuple& v_tuple, const int64_t tag)
{
    assert(v_tuple.is_valid(*this));
    vertex_attrs[v_tuple.vid(*this)].tag = tag;
}

void SimplicialEmbeddingTriMesh::set_edge_tag(const Tuple& e_tuple, const int64_t tag)
{
    assert(e_tuple.is_valid(*this));
    edge_attrs[e_tuple.eid(*this)].tag = tag;

    const size_t v0 = e_tuple.vid(*this);
    const size_t v1 = e_tuple.switch_vertex(*this).vid(*this);
    vertex_attrs[v0].tag = tag;
    vertex_attrs[v1].tag = tag;
}

void SimplicialEmbeddingTriMesh::set_face_tag(const Tuple& f_tuple, const int64_t tag)
{
    assert(f_tuple.is_valid(*this));
    face_attrs[f_tuple.fid(*this)].tag = tag;

    const auto vs = oriented_tri_vertices(f_tuple);
    for (const Tuple& t : vs) {
        vertex_attrs[t.vid(*this)].tag = tag;
        edge_attrs[t.eid(*this)].tag = tag;
    }
}

void SimplicialEmbeddingTriMesh::cache_edge(const Tuple& t)
{
    const size_t v0 = t.vid(*this);
    const size_t v1 = t.switch_vertex(*this).vid(*this);
    const auto& va0 = vertex_attrs[v0];
    const auto& va1 = vertex_attrs[v1];

    const size_t e = t.eid(*this);
    const auto& ea = edge_attrs[e];

    EdgeSplitCache& cache = position_cache.local();
    cache.v0.pos = va0.pos;
    cache.v1.pos = va1.pos;
    cache.e.tag = ea.tag;

    const size_t f0 = t.fid(*this);
    const auto& fa0 = face_attrs[f0];
    cache.f0.tag = fa0.tag;

    const std::optional<Tuple> t_opp = t.switch_face(*this);
    if (t_opp) {
        const size_t f1 = t_opp.value().fid(*this);
        const auto& fa1 = face_attrs[f1];
        cache.f1 = FaceAttributes();
        cache.f1.value().tag = fa1.tag;
    } else {
        cache.f1.reset();
    }
}

bool SimplicialEmbeddingTriMesh::invariants(const std::vector<Tuple>& new_tris)
{
    return true;
}

std::vector<TriMesh::Tuple> SimplicialEmbeddingTriMesh::new_edges_after(
    const std::vector<TriMesh::Tuple>& tris) const
{
    std::vector<TriMesh::Tuple> new_edges;

    for (auto t : tris) {
        for (auto j = 0; j < 3; j++) {
            new_edges.push_back(tuple_from_edge(t.fid(*this), j));
        }
    }
    wmtk::unique_edge_tuples(*this, new_edges);
    return new_edges;
}

std::vector<TriMesh::Tuple> SimplicialEmbeddingTriMesh::replace_edges_after_split(
    const std::vector<TriMesh::Tuple>& tris,
    const size_t vid_threshold) const
{
    // For edge split, we do not immediately push back split sub-edges
    // only push back those edges which are already present, but invalidated by hash mechanism.
    std::vector<TriMesh::Tuple> new_edges;
    new_edges.reserve(tris.size());
    for (auto t : tris) {
        auto tmptup = (t.switch_vertex(*this)).switch_edge(*this);
        if (tmptup.vid(*this) < vid_threshold &&
            (tmptup.switch_vertex(*this)).vid(*this) < vid_threshold)
            new_edges.push_back(tmptup);
    }
    return new_edges;
}

std::vector<TriMesh::Tuple> SimplicialEmbeddingTriMesh::new_sub_edges_after_split(
    const std::vector<TriMesh::Tuple>& tris) const
{
    // only push back the renewed original edges
    std::vector<TriMesh::Tuple> new_edges2;
    new_edges2.reserve(tris.size() * 3);
    for (auto t : tris) {
        new_edges2.push_back(t);
        new_edges2.push_back(t.switch_edge(*this));
        new_edges2.push_back((t.switch_vertex(*this)).switch_edge(*this));
    }

    wmtk::unique_edge_tuples(*this, new_edges2);
    return new_edges2;
}


bool SimplicialEmbeddingTriMesh::split_edge_before(const Tuple& t)
{
    if (!TriMesh::split_edge_before(t)) {
        return false;
    }
    cache_edge(t);
    return true;
}

bool SimplicialEmbeddingTriMesh::split_edge_after(const TriMesh::Tuple& t)
{
    const EdgeSplitCache& cache = position_cache.local();

    const Tuple& e_spine_left = t;
    const Tuple e_rib_0 = t.switch_vertex(*this).switch_edge(*this);
    const Tuple& f_rib_0_left = t;
    const Tuple f_rib_0_right = e_rib_0.switch_face(*this).value();
    const Tuple e_spine_right = f_rib_0_right.switch_edge(*this);

    const size_t vid_spine = t.switch_vertex(*this).vid(*this);

    // set position
    {
        const Vector2d p = 0.5 * (cache.v0.pos + cache.v1.pos);
        vertex_attrs[vid_spine].pos = p;
    }
    // update tags spine
    {
        const auto& e_tag = cache.e.tag;
        vertex_attrs[vid_spine].tag = e_tag;
        edge_attrs[e_spine_left.eid(*this)].tag = e_tag;
        edge_attrs[e_spine_right.eid(*this)].tag = e_tag;
    }
    // update tags rib 0
    {
        const auto& f0_tag = cache.f0.tag;
        edge_attrs[e_rib_0.eid(*this)].tag = f0_tag;
        face_attrs[f_rib_0_left.fid(*this)].tag = f0_tag;
        face_attrs[f_rib_0_right.fid(*this)].tag = f0_tag;
    }
    // update tags rib 1
    if (cache.f1) {
        const Tuple f_rib_1_left = e_spine_left.switch_face(*this).value();
        const Tuple e_rib_1 = f_rib_1_left.switch_vertex(*this).switch_edge(*this);
        const Tuple f_rib_1_right = e_spine_right.switch_face(*this).value();

        const auto& f1_tag = cache.f1.value().tag;
        edge_attrs[e_rib_1.eid(*this)].tag = f1_tag;
        face_attrs[f_rib_1_left.fid(*this)].tag = f1_tag;
        face_attrs[f_rib_1_right.fid(*this)].tag = f1_tag;
    }
    return true;
}

double SimplicialEmbeddingTriMesh::compute_edge_cost_split(const TriMesh::Tuple& t, double L) const
{
    double l =
        (vertex_attrs[t.vid(*this)].pos - vertex_attrs[t.switch_vertex(*this).vid(*this)].pos)
            .norm();
    if (l > (4. / 3.) * L) return (l - (4. / 3.) * L);
    return -1;
}

bool SimplicialEmbeddingTriMesh::split_remeshing(double L)
{
    auto collect_all_ops = std::vector<std::pair<std::string, Tuple>>();
    size_t vid_threshold = 0;
    std::atomic_int count_success = 0;
    auto collect_tuples = tbb::concurrent_vector<Tuple>();

    for_each_edge([&](auto& tup) { collect_tuples.emplace_back(tup); });
    collect_all_ops.reserve(collect_tuples.size());
    for (auto& t : collect_tuples) collect_all_ops.emplace_back("edge_split", t);

    wmtk::logger().info("size for edges to be split is {}", collect_all_ops.size());
    auto edges2 = tbb::concurrent_vector<std::pair<std::string, TriMesh::Tuple>>();
    auto setup_and_execute = [&](auto& executor) {
        vid_threshold = vert_capacity();
        executor.num_threads = NUM_THREADS;
        executor.renew_neighbor_tuples = [&](auto& m, auto op, auto& tris) {
            count_success++;
            auto edges = m.replace_edges_after_split(tris, vid_threshold);
            for (auto e2 : m.new_sub_edges_after_split(tris)) edges2.emplace_back(op, e2);
            auto optup = std::vector<std::pair<std::string, TriMesh::Tuple>>();
            for (auto& e : edges) optup.emplace_back(op, e);
            return optup;
        };
        executor.priority = [&](auto& m, auto _, auto& e) {
            return m.compute_edge_cost_split(e, L);
        };
        executor.should_renew = [](auto val) { return (val > 0); };
        executor.is_weight_up_to_date = [](auto& m, auto& ele) {
            auto& [val, op, e] = ele;
            if (val < 0) return false;
            return true;
        };
        // Execute!!
        do {
            count_success.store(0, std::memory_order_release);
            executor(*this, collect_all_ops);
            collect_all_ops.clear();
            for (auto& item : edges2) collect_all_ops.emplace_back(item);
            edges2.clear();
        } while (count_success.load(std::memory_order_acquire) > 0);
    };

    if (NUM_THREADS > 0) {
        auto executor =
            wmtk::ExecutePass<SimplicialEmbeddingTriMesh, ExecutionPolicy::kPartition>();
        executor.lock_vertices = edge_locker;
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<SimplicialEmbeddingTriMesh, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }

    return true;
}

bool SimplicialEmbeddingTriMesh::uniform_remeshing(double L, int iterations)
{
    int cnt = 0;
    wmtk::logger().info("target len is: {}", L);

    while (cnt < iterations) {
        cnt++;
        wmtk::logger().info("??? Pass ??? {}", cnt);
        // split
        split_remeshing(L);
    }
    wmtk::logger().info("finished {} remeshing iterations", iterations);
    wmtk::logger().info("+++++++++finished+++++++++");
    return true;
}

void SimplicialEmbeddingTriMesh::write(const std::filesystem::path& filename) const
{
    const SimplicialEmbeddingTriMesh& m = *this;

    io::TriVTUWriter writer(m);
    writer.add_vertex_positions([&m](int i) { return m.vertex_attrs[i].pos; });

    writer.add_vertex_attribute("vid", [&m](int i) { return i; });
    writer.add_vertex_attribute("v_tag", [&m](int i) { return m.vertex_attrs[i].tag; });

    writer.add_edge_attribute("eid", [&m](int i) { return i; });
    writer.add_edge_attribute("e_tag", [&m](int i) { return m.edge_attrs[i].tag; });

    writer.add_triangle_attribute("fid", [&m](int i) { return i; });
    writer.add_triangle_attribute("f_tag", [&m](int i) { return m.face_attrs[i].tag; });

    writer.write_triangles(filename.string() + "_f.vtu");
    writer.write_edges(filename.string() + "_e.vtu");
}

} // namespace wmtk::components::simplicial_embedding