#pragma once

#include <wmtk/Scheduler.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::components::internal {

class TaubinSmoothingWithinScalffold
{
    MeshAttributeHandle<double> m_position_handle;
    MeshAttributeHandle<double> m_laplacian_vector_handle;
    MeshAttributeHandle<long> m_vertex_tag_handle;
    MeshAttributeHandle<long> m_vertex_todo_handle;
    long m_input_vertex_tag_value;
    long m_scalffold_vertex_tag_value;
    // the shrink_alpha and inflate_alpha is used in this material
    // https://graphics.stanford.edu/courses/cs468-01-fall/Papers/taubin-smoothing.pdf
    double m_shrink_alpha = 0.3;
    double m_inflate_alpha = 0.31;

public:
    TaubinSmoothingWithinScalffold(
        MeshAttributeHandle<double>& position_handle,
        MeshAttributeHandle<double>& laplacian_vector_handle,
        MeshAttributeHandle<long>& vertex_tag_handle,
        MeshAttributeHandle<long>& vertex_todo_handle,
        const long input_vertex_tag_value,
        const long scalffold_vertex_tag_value,
        const double shrink_alpha,
        const double inflate_alpha);
    void process(TriMesh& m_mesh, const long iterations);
    void process(TetMesh& m_mesh, const long iterations);
};

} // namespace wmtk::components::internal
