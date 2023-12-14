#pragma once
#include <wmtk/Primitive.hpp>

namespace wmtk {
namespace function {
class AsdPerSimplexFunction
{
public:
    AsdPerSimplexFunction() {}
    virtual ~AsdPerSimplexFunction() {}

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
};
} // namespace function
} // namespace wmtk
