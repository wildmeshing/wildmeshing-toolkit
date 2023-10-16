#pragma once
#include <wmtk/attribute/AttributeHandle.hpp>
#include "PerSimplexFunction.hpp"
namespace wmtk {
namespace function {
class DifferentiablePerSimplexFunction : public PerSimplexFunction
{
public:
    DifferentiablePerSimplexFunction(const Mesh& mesh, const Simplex::Type& simplex_type);
    virtual ~DifferentiablePerSimplexFunction();

public:
    virtual Eigen::VectorXd get_gradient(const Simplex& s) const;
    virtual Eigen::MatrixXd get_hessian(const Simplex& s) const;

    const MeshAttributeHandle<double>& get_simplex_attribute_handle() const;

    void assert_function_type(const Simplex::type& s_type) const;

private:
    const MeshAttributeHandle<double> m_attribute_handle;
};
} // namespace function
} // namespace wmtk