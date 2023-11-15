#include "AutodiffFunction.hpp"
#include <wmtk/function/utils/AutoDiffRAII.hpp>
#include <wmtk/simplex/internal/SimplexEqualFunctor.hpp>
namespace wmtk::function {

AutodiffFunction::AutodiffFunction(
    const Mesh& mesh,
    const PrimitiveType& domain_simplex_type,
    const MeshAttributeHandle<double>& variable_attribute_handle)
    : PerSimplexDifferentiableFunction(mesh, domain_simplex_type, variable_attribute_handle)
{}

AutodiffFunction::~AutodiffFunction() = default;

} // namespace wmtk::function
