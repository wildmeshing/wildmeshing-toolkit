#pragma once

#include <igl/Timer.h>
#include <wmtk/TriMesh.h>
#include <wmtk/utils/Morton.h>
#include <wmtk/utils/PartitionMesh.h>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/simplex/RawSimplex.hpp>

// clang-format off
#include <wmtk/utils/DisableWarnings.hpp>
#include <tbb/concurrent_map.h>
#include <tbb/parallel_sort.h>
#include <wmtk/utils/EnableWarnings.hpp>
// clang-format on

namespace wmtk::components::tet_remeshing::surf {

class VertexAttributes
{
public:
    Vector3d m_posf;

    bool m_is_on_edge = false;
    bool m_is_freeze = false;

    size_t partition_id = 0;

    VertexAttributes() {};
    VertexAttributes(const Vector3d& p);
};

class EdgeAttributes
{
public:
    bool m_is_on_edge = false;

    EdgeAttributes() {};
};

class SurfaceMesh : public wmtk::TriMesh
{
public:
    wmtk::AttributeCollection<VertexAttributes> vertex_attrs;
    wmtk::AttributeCollection<EdgeAttributes> edge_attrs;
    // wmtk::AttributeCollection<FaceAttributes> face_attrs;

    SurfaceMesh(const MatrixXd& V, const MatrixXi& F, int num_threads = 0);

    void get_surface(MatrixXd& V, MatrixXi& F) const;
    void get_edge_mesh(MatrixXd& V, MatrixXi& E) const;

    void partition_mesh()
    {
        auto m_vertex_partition_id = partition_TriMesh(*this, NUM_THREADS);
        for (auto i = 0; i < m_vertex_partition_id.size(); i++)
            vertex_attrs[i].partition_id = m_vertex_partition_id[i];
    }

    // TODO: This should not be exposed to the application, but hidden in wmtk
    void partition_mesh_morton();

    void smooth_surface();
    void smooth_edges();
    void smooth_all(const size_t num_iterations);

    bool smooth_before(const Tuple& t) override;
    bool smooth_after(const Tuple& t) override;

    void write_vtu(const std::string& path);
};

} // namespace wmtk::components::tet_remeshing::surf