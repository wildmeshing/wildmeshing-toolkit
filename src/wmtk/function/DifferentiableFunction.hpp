#pragma once
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/AttributeHandle.hpp>
#include "Function.hpp"
namespace wmtk::function {

class DifferentiableFunction : public Function
{
public:
    DifferentiableFunction(
        const Mesh& mesh,
        const MeshAttributeHandle<double>& vertex_attribute_handle);

    virtual ~DifferentiableFunction() = default;


    virtual Eigen::VectorXd get_gradient(const Tuple& tuple) const = 0;
    virtual Eigen::MatrixXd get_hessian(const Tuple& tuple) const = 0;
    const MeshAttributeHandle<double> get_vertex_attribute_handle() const;

    // The dimension that this differentiable function lies in
    // (this is the dimension of the vector space the vertex attribute lies in)
    long embedded_dimension() const;


private:
    const MeshAttributeHandle<double> m_vertex_attribute_handle;
};
} // namespace wmtk::function
