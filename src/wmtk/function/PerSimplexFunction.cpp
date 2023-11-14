#include "PerSimplexFunction.hpp"

namespace wmtk::function {

PerSimplexFunction::PerSimplexFunction(const Mesh& mesh, const PrimitiveType& domain_simplex_type)
    : m_mesh(mesh)
    , m_domain_simplex_type(domain_simplex_type)
{}

PerSimplexFunction::~PerSimplexFunction() = default;

const Mesh& PerSimplexFunction::mesh() const
{
    return m_mesh;
}

PrimitiveType PerSimplexFunction::get_domain_simplex_type() const
{
    return m_domain_simplex_type;
}

double PerSimplexFunction::get_value_sum(const std::vector<Simplex>& simplices) const
{
    double v = 0;
    for (const Simplex& cell : simplices) {
        v += get_value(cell);
    }
    return v;
}

} // namespace wmtk::function
