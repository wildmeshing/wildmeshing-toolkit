#include "DifferentiableFunction.hpp"
#include <wmtk/simplex/top_level_cofaces.hpp>
namespace wmtk::function {

DifferentiableFunction::DifferentiableFunction(
    const Mesh& mesh,
    const MeshAttributeHandle<double>& vertex_attribute_handle)
    : Function(mesh)
    , m_coordinate_vertex_attribute_handle(vertex_attribute_handle)
{}


Eigen::VectorXd DifferentiableFunction::get_one_ring_gradient(const Tuple& vertex) const
{
    auto simplices =
        simplex::top_level_cofaces_tuples(mesh(), Simplex(PrimitiveType::Vertex, vertex));
    return get_gradient_sum(vertex, simplices);
}
Eigen::MatrixXd DifferentiableFunction::get_one_ring_hessian(const Tuple& vertex) const
{
    auto simplices =
        simplex::top_level_cofaces_tuples(mesh(), Simplex(PrimitiveType::Vertex, vertex));
    return get_hessian_sum(vertex, simplices);
}
Eigen::VectorXd DifferentiableFunction::get_gradient_sum(
    const Tuple& vertex,
    const std::vector<Tuple>& top_level_simplices) const
{
    Eigen::VectorXd v = Eigen::VectorXd::Zero(embedded_dimension());
    for (const Tuple& cell : top_level_simplices) {
        v += get_gradient(cell);
    }
    return v;
}
Eigen::MatrixXd DifferentiableFunction::get_hessian_sum(
    const Tuple& vertex,
    const std::vector<Tuple>& top_level_simplices) const
{
    Eigen::MatrixXd v = Eigen::MatrixXd::Zero(embedded_dimension(), embedded_dimension());
    for (const Tuple& cell : top_level_simplices) {
        v += get_hessian(cell);
    }
    return v;
}

const MeshAttributeHandle<double>& DifferentiableFunction::get_vertex_attribute_handle() const
{
    return get_coordinate_attribute_handle();
}
const MeshAttributeHandle<double>& DifferentiableFunction::get_coordinate_attribute_handle() const
{
    return m_coordinate_vertex_attribute_handle;
}
long DifferentiableFunction::embedded_dimension() const
{
    return mesh().get_attribute_dimension(get_coordinate_attribute_handle());
}
} // namespace wmtk::function
