#pragma once
#include <wmtk/function/PerSimplexAutodiffFunction.hpp>
namespace wmtk::function {
/**
 * @brief This is the implementation of the Triangle Area function of a triangle mesh.
 * It uses autodiff encoding for differentiations.
 *
 */
class TriangleAreaAD : public PerSimplexAutodiffFunction
{
public:
    TriangleAreaAD(
        const TriMesh& mesh,
        const attribute::MeshAttributeHandle& vertex_attribute_handle);
    ~TriangleAreaAD();

protected:
    DScalar eval(const simplex::Simplex& domain_simplex, const std::vector<DSVec>& coordinates)
        const override;
};

} // namespace wmtk::function
