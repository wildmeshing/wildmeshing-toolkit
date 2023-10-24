#include "TriMeshValenceFunction.hpp"
#include <wmtk/Primitive.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
namespace wmtk::function {
TriMeshValenceFunction::TriMeshValenceFunction(std::unique_ptr<ValenceEnergyPerEdge>&& function)
    : Function(std::move(function))
{}

double TriMeshValenceFunction::get_value(const Simplex& simplex) const
{
    return m_function->get_value(simplex);
}


} // namespace wmtk::function
