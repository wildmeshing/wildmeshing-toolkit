#include "LocalNeighborsFunction.hpp"


namespace wmtk::function {

LocalNeighborsFunction::LocalNeighborsFunction(std::shared_ptr<PerSimplexFunction> f)
    : m_function(std::move(f))
{}

std::vector<simplex::Simplex> LocalNeighborsFunction::domain(
    const simplex::Simplex& variable_simplex) const
{}
} // namespace wmtk::function
