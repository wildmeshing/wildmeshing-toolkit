#pragma once

#include "Function.hpp"

namespace wmtk::function {

class PerSimplexFunction;

// Specifies a sort of function that combines the results of evaluating a per-simplex function on
// the local neighborhood of some simplex
class LocalNeighborsFunction : public DifferentiableFunction
{
public:
    LocalNeighborsFunction(std::shared_ptr<PerSimplexFunction> f);

    // default implementation is to take the set of cofaces or faces of the variable simplex
    virtual std::vector<simplex::Simplex> domain(const simplex::Simplex& variable_simplex) const;

private:
    std::shared_ptr<PerSimplexFunction> m_function;
};
} // namespace wmtk::function
