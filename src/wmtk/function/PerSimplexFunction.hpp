#pragma once

#include <wmtk/Primitive.hpp>

#include <wmtk/attribute/MeshAttributes.hpp>

#include <Eigen/Core>

namespace wmtk::function {

class PerSimplexFunction
{
public:
    PerSimplexFunction(
        const Mesh& mesh,
        const PrimitiveType primitive_type,
        const attribute::MeshAttributeHandle<double>& variable_attribute_handle);
    virtual ~PerSimplexFunction() {}

    /**
     * @brief This function is defined over a simplex (normally a triangle or tetrahedron). And the
     * domain of the function is represented by the input argument domain_simplex.
     *
     * @param domain_simplex The domain that the function is defined over.
     * @return double The numerical value of the function at the input domain.
     */
    virtual double get_value(const simplex::Simplex& domain_simplex) const = 0;
    virtual Eigen::VectorXd get_gradient(
        const simplex::Simplex& domain_simplex,
        const simplex::Simplex& variable_simplex) const
    {
        throw std::runtime_error("Gradient not implemented");
    }
    virtual Eigen::MatrixXd get_hessian(
        const simplex::Simplex& domain_simplex,
        const simplex::Simplex& variable_simplex) const
    {
        throw std::runtime_error("Hessian not implemented");
    }

    inline const Mesh& mesh() const { return m_mesh; }
    inline const MeshAttributeHandle<double>& attribute_handle() const
    {
        assert(m_handle.is_valid());
        return m_handle;
    }

    long embedded_dimension() const;

private:
    MeshAttributeHandle<double> m_handle;
    const Mesh& m_mesh;

protected:
    const PrimitiveType m_primitive_type;
};
} // namespace wmtk::function
