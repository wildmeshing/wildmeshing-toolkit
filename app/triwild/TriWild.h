#pragma once

#include <igl/Timer.h>
#include <wmtk/TriMesh.h>
#include "Parameters.h"


namespace triwild {

class VertexAttributes
{
public:
    Eigen::Vector2d pos;

    size_t partition_id = 0; // TODO this should not be here

    // Vertices marked as fixed cannot be modified by any local operation
    bool fixed = false;
};


class FaceAttributes
{
public:
};

class TriWild : public wmtk::TriMesh
{
public:
    // Energy Assigned to undefined energy
    // TODO: why not the max double?
    const double MAX_ENERGY = 1e50;
    double target_l = -1.; // targeted edge length
    double target_lr = 5e-2; // targeted relative edge length

    TriWild(double _target_l = -1.) { target_l = _target_l; };

    virtual ~TriWild(){};

    // Store the per-vertex attributes
    wmtk::AttributeCollection<VertexAttributes> vertex_attrs;
    struct InfoCache
    {
        size_t v1;
        size_t v2;
        double max_energy;
        int partition_id;
    };
    tbb::enumerable_thread_specific<InfoCache> cache;

    // Initializes the mesh
    void create_mesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F);

    // Exports V and F of the stored mesh
    void export_mesh(Eigen::MatrixXd& V, Eigen::MatrixXi& F);

    // Writes a triangle mesh in OBJ format
    void write_obj(const std::string& path);

    // Computes the quality of a triangle
    double get_quality(const Tuple& loc) const;

    // Computes the average quality of a mesh
    Eigen::VectorXd get_quality_all_triangles();

    // Check if a triangle is inverted
    bool is_inverted(const Tuple& loc) const;

    std::vector<TriMesh::Tuple> new_edges_after(const std::vector<TriMesh::Tuple>& tris) const;

    // Smoothing
    void smooth_all_vertices();
    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    // Collapse
    void collapse_all_edges();
    bool collapse_edge_before(const Tuple& t) override;
    bool collapse_edge_after(const Tuple& t) override;
    // Split
    void split_all_edges();
    bool split_edge_before(const Tuple& t) override;
    bool split_edge_after(const Tuple& t) override;
    // Swap
    void swap_all_edges();
    bool swap_edge_before(const Tuple& t) override;
    bool swap_edge_after(const Tuple& t) override;

    void mesh_improvement(int max_its);
    double get_length2(const Tuple& t) const;
};

} // namespace triwild
