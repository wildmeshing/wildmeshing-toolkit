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
        const Simplex& domain_simplex,
        const Simplex& variable_simplex) const = 0;
    virtual Eigen::MatrixXd get_hessian(
        const Simplex& domain_simplex,
        const Simplex& variable_simplex) const = 0;

private:
    MeshAttributeHandle<double> m_coordinate_attribute_handle;
    const Mesh& m_mesh;

protected:
    inline MeshAttributeHandle<double> get_coordinate_attribute_handle() const
    {
        return m_coordinate_attribute_handle;
    }

    inline const Mesh& mesh() const { return m_mesh; }
    long embedded_dimension() const;
};
} // namespace wmtk::function
