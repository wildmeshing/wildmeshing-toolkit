#include "PerSimplexDifferentiableFunction.hpp"

namespace wmtk::function {
PerSimplexDifferentiableFunction::PerSimplexDifferentiableFunction(
    const Mesh& mesh,
    PrimitiveType simplex_type,
    const attribute::MeshAttributeHandle<double>& variable_attribute_handle)
    : PerSimplexFunction(mesh, simplex_type)
    , m_coordinate_attribute_handle(variable_attribute_handle)
{}

PerSimplexDifferentiableFunction::~PerSimplexDifferentiableFunction() = default;

MeshAttributeHandle<double>
PerSimplexDifferentiableFunction::get_coordinate_attribute_handle() const
{
    return m_coordinate_attribute_handle;
}

long PerSimplexDifferentiableFunction::embedded_dimension() const
{
    return DifferentiableFunction::embedded_dimension();
}

Eigen::VectorXd PerSimplexDifferentiableFunction::get_gradient(const Simplex& s) const
{
    assert(get_simplex_type() == s.primitive_type());
    return get_gradient(s.tuple());
}
Eigen::MatrixXd PerSimplexDifferentiableFunction::get_hessian(const Simplex& s) const
{
    assert(get_simplex_type() == s.primitive_type());
    return get_hessian(s.tuple());
}

Eigen::VectorXd PerSimplexDifferentiableFunction::get_gradient_sum(
    const std::vector<Simplex>& simplices) const
{
    Eigen::VectorXd g = Eigen::VectorXd::Zero(embedded_dimension());
    for (const Simplex& cell : simplices) {
        g += get_gradient(cell);
    }
    return g;
}
Eigen::MatrixXd PerSimplexDifferentiableFunction::get_hessian_sum(
    const std::vector<Simplex>& simplices) const
{
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(embedded_dimension(), embedded_dimension());
    for (const Simplex& cell : simplices) {
        h += get_hessian(cell);
    }
    return h;
}

Eigen::VectorXd PerSimplexDifferentiableFunction::get_gradient_sum(
    const std::vector<Tuple>& tuples) const
{
    Eigen::VectorXd g = Eigen::VectorXd::Zero(embedded_dimension());
    for (const Tuple& t : tuples) {
        g += get_gradient(t);
    }
    return g;
}
Eigen::MatrixXd PerSimplexDifferentiableFunction::get_hessian_sum(
    const std::vector<Tuple>& tuples) const
{
    Eigen::MatrixXd h = Eigen::MatrixXd::Zero(embedded_dimension(), embedded_dimension());
    for (const Tuple& t : tuples) {
        h += get_hessian(t);
    }
    return h;
}
} // namespace wmtk::function
