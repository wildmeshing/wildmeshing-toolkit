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
    std::array<DSVec, 3> get_variable_coordinates(
        const simplex::Simplex& domain_simplex,
        const std::optional<simplex::Simplex>& variable_simplex_opt) const;
};
} // namespace wmtk::function