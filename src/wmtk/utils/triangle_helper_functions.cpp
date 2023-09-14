#include "triangle_helper_functions.hpp"
using namespace wmtk;

double triangle_2d_area(
    const TriMesh& m,
    const MeshAttributeHandle<double>& vertex_uv_handle,
    const Tuple& tuple)
{
    // assuming traingle is ccw
    ConstAccessor<double> pos = m.create_const_accessor(vertex_uv_handle);
    Eigen::Vector2d p0 = pos.const_vector_attribute(tuple);
    Eigen::Vector2d p1 = pos.const_vector_attribute(m.switch_edge(m.switch_vertex(tuple)));
    Eigen::Vector2d p2 = pos.const_vector_attribute(m.switch_vertex(tuple));
    return triangle_2d_area<double>(p0, p1, p2);
}