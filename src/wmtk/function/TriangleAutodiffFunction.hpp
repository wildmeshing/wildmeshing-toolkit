#include <wmtk/TriMesh.hpp>
#include "AutodiffFunction.hpp"
namespace wmtk::function {
/**
 * @brief This is an extension of the AutodiffFunction class where the domain is restricted to be on
 * a triangle mesh
 *
 */
class TriangleAutodiffFunction : public AutodiffFunction
{
public:
    TriangleAutodiffFunction(
        const TriMesh& mesh,
        const attribute::MeshAttributeHandle<double>& coordinate_attribute_handle);
    ~TriangleAutodiffFunction();

    using DSVec = Eigen::VectorX<DScalar>;

public:
    double get_value(const simplex::Simplex& domain_simplex) const override;

    Eigen::VectorXd get_gradient(
        const simplex::Simplex& domain_simplex,
        const simplex::Simplex& variable_simplex) const override;

    Eigen::MatrixXd get_hessian(
        const simplex::Simplex& domain_simplex,
        const simplex::Simplex& variable_simplex) const override;

protected:
    std::array<DSVec, 3> get_coordinates(
        const simplex::Simplex& domain_simplex,
        const std::optional<simplex::Simplex>& variable_simplex_opt = {}) const;

    /**
     * @brief This function defines a function f(x) where f is defined over a triangle domain and
     * the variables for f are the three vertices coordinates of the triangle.
     *
     * The input three coordinates are obtained through the get_coordinates function. Theya
     * re encoded in autodiff type so that the function can be differentiated through autodiff.
     *
     * @param coordinate0
     * @param coordinate1
     * @param coordinate2
     * @return DScalar
     */
    virtual DScalar eval(
        const simplex::Simplex& domain_simplex,
        DSVec& coordinate0,
        DSVec& coordinate1,
        DSVec& coordinate2) const = 0;
};
} // namespace wmtk::function
