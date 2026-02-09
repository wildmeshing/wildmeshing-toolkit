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

struct
{
    bool operator()(SimplicialEmbeddingTriMesh& m, const TriMesh::Tuple& f, int task_id)
    {
        // TODO: this should not be here
        return m.try_set_face_mutex_one_ring(f, task_id);
    }
} face_locker;

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

    for (size_t i = 0; i < vertex_positions.size(); i++) {
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

bool SimplicialEmbeddingTriMesh::invariants(const std::vector<Tuple>& new_tris)
{
    return true;
}

bool SimplicialEmbeddingTriMesh::split_edge_before(const Tuple& t)
{
    using namespace simplex;

    const Tuple v1_tuple = t.switch_vertex(*this);

    const size_t v0id = t.vid(*this);
    const size_t v1id = v1_tuple.vid(*this);
    const auto& va0 = vertex_attrs[v0id];
    const auto& va1 = vertex_attrs[v1id];

    const size_t eid = t.eid(*this);
    const auto& ea = edge_attrs[eid];

    EdgeSplitCache& cache = edge_split_cache.local();
    cache.v0.pos = va0.pos;
    cache.v1.pos = va1.pos;
    cache.e.tag = ea.tag;

    const Edge e(v0id, v1id);
    const auto ef = simplex_incident_triangles(e);

    cache.face_infos.clear();
    for (const Face& f : ef.faces()) {
        EdgeSplitPerFaceCache c;
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

    return true;
}

bool SimplicialEmbeddingTriMesh::split_edge_after(const TriMesh::Tuple& t)
{
    using namespace simplex;

    const EdgeSplitCache& cache = edge_split_cache.local();

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

bool SimplicialEmbeddingTriMesh::split_face_before(const Tuple& t)
{
    FaceSplitCache& cache = face_split_cache.local();

    const Tuple& t0 = t;
    const Tuple t1 = t0.switch_vertex(*this).switch_edge(*this);
    const Tuple t2 = t1.switch_vertex(*this).switch_edge(*this);

    cache.v0 = vertex_attrs[t0.vid(*this)];
    cache.v1 = vertex_attrs[t1.vid(*this)];
    cache.v2 = vertex_attrs[t2.vid(*this)];

    cache.e0 = edge_attrs[t0.eid(*this)];
    cache.e1 = edge_attrs[t1.eid(*this)];
    cache.e2 = edge_attrs[t2.eid(*this)];

    cache.f = face_attrs[t.fid(*this)];

    return true;
}

bool SimplicialEmbeddingTriMesh::split_face_after(const Tuple& t)
{
    const FaceSplitCache& cache = face_split_cache.local();

    const Tuple& t0 = t;
    const Tuple t1 =
        t0.switch_vertex(*this).switch_edge(*this).switch_face(*this).value().switch_edge(*this);
    const Tuple t2 =
        t1.switch_vertex(*this).switch_edge(*this).switch_face(*this).value().switch_edge(*this);

    // vertex_attrs[t0.vid(*this)] = cache.v0;
    // vertex_attrs[t1.vid(*this)] = cache.v1;
    // vertex_attrs[t2.vid(*this)] = cache.v2;

    edge_attrs[t0.eid(*this)] = cache.e0;
    edge_attrs[t1.eid(*this)] = cache.e1;
    edge_attrs[t2.eid(*this)] = cache.e2;

    face_attrs[t0.fid(*this)] = cache.f;
    face_attrs[t1.fid(*this)] = cache.f;
    face_attrs[t2.fid(*this)] = cache.f;

    // new vertex
    const size_t new_vid = t.switch_edge(*this).switch_vertex(*this).vid(*this);
    vertex_attrs[new_vid].tag = cache.f.tag;
    vertex_attrs[new_vid].pos = (cache.v0.pos + cache.v1.pos + cache.v2.pos) / 3.;

    // new edges
    edge_attrs[t0.switch_edge(*this).eid(*this)].tag = cache.f.tag;
    edge_attrs[t1.switch_edge(*this).eid(*this)].tag = cache.f.tag;
    edge_attrs[t2.switch_edge(*this).eid(*this)].tag = cache.f.tag;

    return true;
}

bool SimplicialEmbeddingTriMesh::edge_needs_split(const Tuple& t) const
{
    const size_t v0 = t.vid(*this);
    const size_t v1 = t.switch_vertex(*this).vid(*this);
    const size_t e = t.eid(*this);
    if (vertex_attrs[v0].tag == 1 && vertex_attrs[v1].tag == 1 && edge_attrs[e].tag == 0) {
        return true;
    }
    return false;
}

bool SimplicialEmbeddingTriMesh::face_needs_split(const Tuple& t) const
{
    const auto [t0, t1, t2] = oriented_tri_vertices(t);

    if (face_attrs[t.fid(*this)].tag == 1) {
        return false;
    }
    if (vertex_attrs[t0.vid(*this)].tag == DEFAULT_TAG ||
        vertex_attrs[t1.vid(*this)].tag == DEFAULT_TAG ||
        vertex_attrs[t2.vid(*this)].tag == DEFAULT_TAG) {
        return false;
    }
    if (edge_attrs[t0.eid(*this)].tag == DEFAULT_TAG ||
        edge_attrs[t1.eid(*this)].tag == DEFAULT_TAG ||
        edge_attrs[t2.eid(*this)].tag == DEFAULT_TAG) {
        return false;
    }
    return true;
}

bool SimplicialEmbeddingTriMesh::edge_split_simplicial_embedding()
{
    auto all_ops = std::vector<std::pair<std::string, Tuple>>();
    {
        auto tuples = tbb::concurrent_vector<Tuple>();
        for (const Tuple& t : get_edges()) {
            if (edge_needs_split(t)) {
                tuples.emplace_back(t);
            }
        }

        all_ops.reserve(tuples.size());
        for (const Tuple& t : tuples) {
            all_ops.emplace_back("edge_split", t);
        }
    }

    wmtk::logger().info("Edge split OPs: {}", all_ops.size());

    auto setup_and_execute = [&](auto& executor) {
        executor.num_threads = NUM_THREADS;
        executor.renew_neighbor_tuples =
            [&](auto& m, auto op, auto& tris) -> std::vector<std::pair<Op, Tuple>> {
            std::vector<std::pair<Op, Tuple>> new_ops;
            new_ops.reserve(tris.size());
            for (const Tuple& t : tris) {
                if (edge_needs_split(t)) {
                    new_ops.emplace_back(op, t);
                }
            }

            return new_ops;
        };
        executor.priority = [](auto& m, auto _, auto& e) {
            const Vector2d& p0 = m.vertex_attrs[e.vid(m)].pos;
            const Vector2d& p1 = m.vertex_attrs[e.switch_vertex(m).vid(m)].pos;
            return (p1 - p0).norm();
        };

        // Execute!!
        executor(*this, all_ops);
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

bool SimplicialEmbeddingTriMesh::face_split_simplicial_embedding()
{
    auto all_ops = std::vector<std::pair<std::string, Tuple>>();
    {
        auto tuples = tbb::concurrent_vector<Tuple>();
        for (const Tuple& t : get_faces()) {
            if (face_needs_split(t)) {
                tuples.emplace_back(t);
            }
        }

        all_ops.reserve(tuples.size());
        for (const Tuple& t : tuples) {
            all_ops.emplace_back("face_split", t);
        }
    }

    wmtk::logger().info("Face split OPs: {}", all_ops.size());

    auto setup_and_execute = [&](auto& executor) {
        executor.num_threads = NUM_THREADS;
        // Execute!!
        executor(*this, all_ops);
    };

    if (NUM_THREADS > 0) {
        auto executor =
            wmtk::ExecutePass<SimplicialEmbeddingTriMesh, ExecutionPolicy::kPartition>();
        executor.lock_vertices = face_locker;
        setup_and_execute(executor);
    } else {
        auto executor = wmtk::ExecutePass<SimplicialEmbeddingTriMesh, ExecutionPolicy::kSeq>();
        setup_and_execute(executor);
    }

    return true;
}

void SimplicialEmbeddingTriMesh::simplicial_embedding()
{
    edge_split_simplicial_embedding();
    face_split_simplicial_embedding();
}

void SimplicialEmbeddingTriMesh::write(const std::filesystem::path& filename) const
{
    const SimplicialEmbeddingTriMesh& m = *this;

    io::TriVTUWriter writer(m);
    writer.add_vertex_positions([&m](size_t i) { return m.vertex_attrs[i].pos; });

    writer.add_vertex_attribute("vid", [&m](size_t i) { return i; });
    writer.add_vertex_attribute("v_tag", [&m](size_t i) { return m.vertex_attrs[i].tag; });

    writer.add_edge_attribute("eid", [&m](size_t i) { return i; });
    writer.add_edge_attribute("e_tag", [&m](size_t i) { return m.edge_attrs[i].tag; });

    writer.add_triangle_attribute("fid", [&m](size_t i) { return i; });
    writer.add_triangle_attribute("f_tag", [&m](size_t i) { return m.face_attrs[i].tag; });

    writer.write_triangles(filename.string() + "_f.vtu");
    writer.write_edges(filename.string() + "_e.vtu");
}

} // namespace wmtk::components::simplicial_embedding