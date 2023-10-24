#include "PerSimplexFunction.hpp"

namespace wmtk {
namespace function {

PerSimplexFunction::PerSimplexFunction(const Mesh& mesh, const PrimitiveType& simplex_type)
    : m_mesh(mesh)
    , m_simplex_type(simplex_type)
{}

PerSimplexFunction::~PerSimplexFunction() = default;

const Mesh& PerSimplexFunction::mesh() const
{
    return m_mesh;
}

const PrimitiveType& PerSimplexFunction::get_simplex_type() const
{
    return m_simplex_type;
}
} // namespace function
}; // namespace wmtk