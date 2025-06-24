#pragma once

#include <wmtk/TriMesh.h>
#include <Eigen/Dense>
#include <filesystem>
#include <wmtk/Types.hpp>

namespace wmtk::components::simplicial_embedding {

constexpr size_t DEFAULT_TAG = 0;

struct VertexAttributes
{
    Vector2d pos;
    int64_t tag = DEFAULT_TAG;
    size_t partition_id = 0;
};
struct EdgeAttributes
{
    int64_t tag = DEFAULT_TAG;
};
struct FaceAttributes
{
    int64_t tag = DEFAULT_TAG;
};

class SimplicialEmbeddingTriMesh : public wmtk::TriMesh
{
public:
    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    using EdgeAttCol = wmtk::AttributeCollection<EdgeAttributes>;
    using FaceAttCol = wmtk::AttributeCollection<FaceAttributes>;
    VertAttCol vertex_attrs;
    EdgeAttCol edge_attrs;
    FaceAttCol face_attrs;

    int retry_limit = 10;
    SimplicialEmbeddingTriMesh();

    void set_positions(const std::vector<Vector2d>& vertex_positions);
    void set_positions(const MatrixXd& V);
    void set_num_threads(const int64_t num_threads);

    void set_vertex_tag(const Tuple& v_tuple, const int64_t tag);
    void set_edge_tag(const Tuple& e_tuple, const int64_t tag);
    void set_face_tag(const Tuple& f_tuple, const int64_t tag);

    struct EdgeSplitPerFaceCache
    {
        VertexAttributes v; // vertex opposite to edge (probably not necessary)
        FaceAttributes f; // face defined by v + e
        EdgeAttributes v0e; // edge v0 + v
        EdgeAttributes v1e; // edge v1 + v
    };
    struct EdgeSplitCache
    {
        VertexAttributes v0, v1; // incident vertices
        EdgeAttributes e; // splitted edge

        std::map<size_t, EdgeSplitPerFaceCache>
            face_infos; // map from link vertex id to per-face cache
    };
    tbb::enumerable_thread_specific<EdgeSplitCache> edge_split_cache;

    struct FaceSplitCache
    {
        VertexAttributes v0, v1, v2;
        EdgeAttributes e0, e1, e2;
        FaceAttributes f;
    };
    tbb::enumerable_thread_specific<FaceSplitCache> face_split_cache;

    void cache_edge(const Tuple& t);

    bool invariants(const std::vector<Tuple>& new_tris) override;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& tris) const;
    std::vector<TriMesh::Tuple> replace_edges_after_split(
        const std::vector<TriMesh::Tuple>& tris,
        const size_t vid_threshold) const;
    std::vector<TriMesh::Tuple> new_sub_edges_after_split(
        const std::vector<TriMesh::Tuple>& tris) const;


    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;

    bool split_face_before(const Tuple& t) override;
    bool split_face_after(const Tuple& t) override;

    bool face_needs_split(const Tuple& t);

    double compute_edge_cost_split(const TriMesh::Tuple& t, double L) const;
    bool edge_split_simplicial_embedding();
    bool face_split_simplicial_embedding();
    bool uniform_remeshing(double L, int interations);

    void write(const std::filesystem::path& filename) const;
};

} // namespace wmtk::components::simplicial_embedding
