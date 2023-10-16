#include "DifferentiablePerSimplexFunction.hpp"

namespace wmtk {
namespace function {
DifferentiablePerSimplexFunction::DifferentiablePerSimplexFunction(
    const Mesh& mesh,
    const Simplex::Type& simplex_type)
    : PerSimplexFunction(mesh, simplex_type)
{}

DifferentiablePerSimplexFunction::~DifferentiablePerSimplexFunction() = default;

const MeshAttributeHandle<double>& DifferentiablePerSimplexFunction::get_simplex_attribute_handle()
    const
{
    return m_attribute_handle;
}

void DifferentiablePerSimplexFunction::assert_function_type(const Simplex::type& s_type) const
{
    if (m_attribute_handle.get_attribute_type() != s_type) {
        throw std::runtime_error("Differentiation of the DifferentiableFunction must be taken wrt "
                                 "the attribute of the simplex type ");
    }
    if (get_simplex_type() > s_type || get_simplex_type() == s_type) {
        throw std::runtime_error("The DifferentiableFunction must be defined on the cofaces of the "
                                 "simplex type ");
    }
}
} // namespace function
} // namespace wmtk