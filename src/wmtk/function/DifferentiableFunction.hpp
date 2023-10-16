#pragma once
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/AttributeHandle.hpp>
#include "Function.hpp"
namespace wmtk::function {

class DifferentiableFunction : public Function
{
public:

    // @param The mesh representing the domain the function is differentiated over
    // @param The mesh's attribute that the function is differentiated against
    DifferentiableFunction(
        const Mesh& mesh,
        const MeshAttributeHandle<double>& coordinate_vertex_attribute_handle);

    virtual ~DifferentiableFunction() = default;


    Eigen::VectorXd get_one_ring_gradient(const Tuple& vertex) const;
    Eigen::MatrixXd get_one_ring_hessian(const Tuple& vertex) const;

    Eigen::VectorXd get_gradient_sum(const Tuple& tuple, const std::vector<Tuple>& top_level_simplices) const;
    Eigen::MatrixXd get_hessian_sum(const Tuple& tuple, const std::vector<Tuple>& top_level_simplices) const;


    const MeshAttributeHandle<double>& get_coordinate_attribute_handle() const;

    // alias for the coordinate_attribute_handle
    const MeshAttributeHandle<double>& get_vertex_attribute_handle() const;
    // The dimension that this differentiable function lies in
    // (this is the dimension of the vector space the vertex attribute lies in)
    long embedded_dimension() const;

protected:
    virtual Eigen::VectorXd get_gradient(const Tuple& top_level_simplex) const = 0;

    // TODO: should differentiable function be required to be twice differentiable?
    virtual Eigen::MatrixXd get_hessian(const Tuple& top_level_simplex) const = 0;

private:
    const MeshAttributeHandle<double> m_coordinate_vertex_attribute_handle;
};
} // namespace wmtk::function
