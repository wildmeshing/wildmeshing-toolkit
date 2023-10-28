#include "PerSimplexFunction.hpp"

namespace wmtk::function {

PerSimplexFunction::PerSimplexFunction(const Mesh& mesh, const PrimitiveType& simplex_type)
    : m_mesh(mesh)
    , m_simplex_type(simplex_type)
{}

PerSimplexFunction::~PerSimplexFunction() = default;

const Mesh& PerSimplexFunction::mesh() const
{
    return m_mesh;
}

PrimitiveType PerSimplexFunction::get_simplex_type() const
{
    return m_simplex_type;
}


double PerSimplexFunction::get_value(const Simplex& s) const
{
    assert(get_simplex_type() == s.primitive_type());
    return get_value(s.tuple());
}
double PerSimplexFunction::get_value_sum(const std::vector<Simplex>& simplices) const
{
    double v = 0;
    for (const Simplex& cell : simplices) {
        v += get_value(cell);
    }
    return v;
}
double PerSimplexFunction::get_value_sum(const std::vector<Tuple>& tuples) const
{
    double v = 0;
    const PrimitiveType pt = get_simplex_type();
    for (const Tuple& tuple : tuples) {
        v += get_value(tuple);
    }
    return v;
}
}; // namespace wmtk::function
