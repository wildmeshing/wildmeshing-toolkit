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
    using namespace simplex;

    const Tuple v1_tuple = t.switch_vertex(*this);

    const size_t v0id = t.vid(*this);
    const size_t v1id = v1_tuple.vid(*this);
    const auto& va0 = vertex_attrs[v0id];
    const auto& va1 = vertex_attrs[v1id];

    const size_t eid = t.eid(*this);
    const auto& ea = edge_attrs[eid];

    EdgeSplitCache& cache = position_cache.local();
    cache.v0.pos = va0.pos;
    cache.v1.pos = va1.pos;
    cache.e.tag = ea.tag;

    const Edge e(v0id, v1id);
    const auto ef = simplex_incident_triangles(e);

    cache.face_infos.clear();
    for (const Face& f : ef.faces()) {
        PerFaceCache c;
        const Vertex v = f.opposite_vertex(e); // link vertex
        c.v = vertex_attrs[v.id()];
        const Tuple f_tuple = tuple_from_simplex(f);
        c.f = face_attrs[f_tuple.fid(*this)];

        const Tuple v0e = tuple_from_vids(v0id, v.id(), v1id);
        const Tuple v1e = tuple_from_vids(v1id, v.id(), v0id);
        c.v0e = edge_attrs[v0e.eid(*this)];
        c.v1e = edge_attrs[v1e.eid(*this)];
        cache.face_infos[v.id()] = c;
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
    using namespace simplex;

    const EdgeSplitCache& cache = position_cache.local();

    const Tuple& v0 = t;
    const Tuple& ee0 = t; // v0 - v_new
    const Tuple v_new = t.switch_vertex(*this);
    const Tuple ee1 =
        v_new.switch_edge(*this).switch_face(*this).value().switch_edge(*this).switch_vertex(
            *this); // v1 - v_new
    const Tuple& v1 = ee1;

    const size_t vid_new = v_new.vid(*this);

    // set position
    {
        const Vector2d p = 0.5 * (cache.v0.pos + cache.v1.pos);
        vertex_attrs[vid_new].pos = p;
    }

    // update tags spine
    {
        const auto& e_tag = cache.e.tag;
        vertex_attrs[vid_new].tag = e_tag;
        edge_attrs[ee0.eid(*this)].tag = e_tag;
        edge_attrs[ee1.eid(*this)].tag = e_tag;
    }

    // update tags on splitted faces
    const size_t v0id = v0.vid(*this);
    const size_t v1id = v1.vid(*this);
    for (const auto& [vid, c] : cache.face_infos) {
        // link vertex
        vertex_attrs[vid] = c.v;
        // faces / edges
        const Tuple t0 = tuple_from_vids(v0id, vid, vid_new);
        face_attrs[t0.fid(*this)] = c.f;
        edge_attrs[t0.eid(*this)] = c.v0e;

        const Tuple t1 = tuple_from_vids(v1id, vid, vid_new);
        face_attrs[t1.fid(*this)] = c.f;
        edge_attrs[t1.eid(*this)] = c.v1e;

        // new edge
        const Tuple te = tuple_from_vids(vid_new, vid, v0id);
        edge_attrs[te.eid(*this)].tag = c.f.tag;
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