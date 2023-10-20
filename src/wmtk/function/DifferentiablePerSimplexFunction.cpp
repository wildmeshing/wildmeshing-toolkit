#include "DifferentiablePerSimplexFunction.hpp"

namespace wmtk {
namespace function {
DifferentiablePerSimplexFunction::DifferentiablePerSimplexFunction(
    const Mesh& mesh,
    const PrimitiveType& simplex_type,
    const attribute::MeshAttributeHandle<double>& variable_attribute_handle)
    : PerSimplexFunction(mesh, simplex_type)
    , m_attribute_handle(variable_attribute_handle)
{}

DifferentiablePerSimplexFunction::~DifferentiablePerSimplexFunction() = default;

const MeshAttributeHandle<double>& DifferentiablePerSimplexFunction::get_variable_attribute_handle()
    const
{
    return m_attribute_handle;
}

void DifferentiablePerSimplexFunction::assert_function_type(const PrimitiveType& s_type) const
{
    if (get_variable_attribute_handle().primitive_type() != s_type) {
        throw std::runtime_error("Differentiation of the DifferentiableFunction must be taken wrt "
                                 "the attribute of the simplex type ");
    }
    if (get_simplex_type() < s_type) {
        throw std::runtime_error(
            "The DifferentiableFunction must be defined on the cofaces of higher dimension to the "
            "simplex type ");
    }
}
} // namespace function
} // namespace wmtk