#pragma once

#include <wmtk/TriMesh.h>
#include <Eigen/Dense>
#include <filesystem>
#include <wmtk/Types.hpp>

namespace wmtk::components::simplicial_embedding {

struct VertexAttributes
{
    Vector2d pos;
    size_t partition_id = 0;
};

class SimplicialEmbeddingTriMesh : public wmtk::TriMesh
{
public:
    using VertAttCol = wmtk::AttributeCollection<VertexAttributes>;
    VertAttCol vertex_attrs;

    int retry_limit = 10;
    SimplicialEmbeddingTriMesh();

    void set_positions(const std::vector<Eigen::Vector2d>& vertex_positions);
    void set_positions(const Eigen::MatrixXd& V);
    void set_num_threads(const int64_t num_threads);

    Eigen::MatrixXi get_F() const;
    Eigen::MatrixXd get_V() const;

    VectorXd position(size_t vid) const { return vertex_attrs[vid].pos; }
    std::vector<VectorXd> serialize_vertex_attributes(size_t vid) const
    {
        const auto& attrs = vertex_attrs[vid];
        return {attrs.pos};
    }
    std::vector<std::string> serialize_vertex_attributes_names() const { return {"position"}; }

    struct PositionInfoCache
    {
        Eigen::Vector2d v1p;
        Eigen::Vector2d v2p;
        int partition_id;
    };
    tbb::enumerable_thread_specific<PositionInfoCache> position_cache;

    void cache_edge_positions(const Tuple& t);

    bool invariants(const std::vector<Tuple>& new_tris) override;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& tris) const;
    std::vector<TriMesh::Tuple> replace_edges_after_split(
        const std::vector<TriMesh::Tuple>& tris,
        const size_t vid_threshold) const;
    std::vector<TriMesh::Tuple> new_sub_edges_after_split(
        const std::vector<TriMesh::Tuple>& tris) const;


    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;

    double compute_edge_cost_split(const TriMesh::Tuple& t, double L) const;
    bool split_remeshing(double L);
    bool uniform_remeshing(double L, int interations);

    bool write_mesh(const std::filesystem::path& filename);
};

} // namespace wmtk::components::simplicial_embedding
