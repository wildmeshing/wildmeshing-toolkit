#include "OneRingSum.hpp"
#include <wmtk/SimplicialComplex.hpp>

namespace wmtk {
namespace function {
OneRingSum::OneRingSum(
    const TriMesh& mesh,
    const MeshAttributeHandle<double>& vertex_attribute_handle,
    const std::unique_ptr<AutodiffFunction>& func)
    : AutodiffFunction(mesh, vertex_attribute_handle)
    , m_func(std::move(func))
{}

double OneRingSum::get_value(const Tuple& tuple) const
{
    // get one-ring opposite edges
    SimplexCollection sc = link(mesh(), Simplex::vertex(t));
    double ret = 0.;
    for (const Simplex& edge : sc.simplex_vector(PrimitiveType::Edge)) {
        Tuple opposite_edge_tuple = edge.tuple();
        ret += m_func->get_value(mesh().switch_edge(mesh().switch_vertex(opposite_edge_tuple)));
    }
    // return the energy
    return ret;
}
Eigen::VectorXd get_gradient(const Tuple& tuple) const
{
    // get one-ring opposite edges
    SimplexCollection sc = link(mesh(), Simplex::vertex(t));
    Eigen::VectorXd ret;
    ret = Eigen::VectorXd::Zero(embedding_dimension());
    for (const Simplex& edge : sc.simplex_vector(PrimitiveType::Edge)) {
        Tuple opposite_edge_tuple = edge.tuple();
        ret += m_func->get_gradient(mesh().switch_edge(mesh().switch_vertex(opposite_edge_tuple)));
    }
    // return the energy
    return ret;
}
Eigen::MatrixXd get_hessian(const Tuple& tuple) const
{
    // get one-ring opposite edges
    SimplexCollection sc = link(mesh(), Simplex::vertex(t));
    Eigen::VectorXd ret;
    ret = Eigen::MatrixXd::Zero(embedding_dimension(), embedding_dimension());
    for (const Simplex& edge : sc.simplex_vector(PrimitiveType::Edge)) {
        Tuple opposite_edge_tuple = edge.tuple();
        ret += m_func->get_hessian(mesh().switch_edge(mesh().switch_vertex(opposite_edge_tuple)));
    }
    // return the energy
    return ret;
}
} // namespace function
} // namespace wmtk