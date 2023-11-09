#pragma once
#include <memory>
#include <wmtk/Primitive.hpp>
#include "PerSimplexFunction.hpp"
namespace wmtk::function {

// a function that invokes a function on a local neighborhood of an input simplex
// Typically we will want to compute something like the gradient of a function defined on the triangles/tetrahedra with respect to a vertex and the choice of basis functions means we only need to compute a one-ring neighborhood.
// This class lets us select a per-simplex function (tet or tri) and evaluate that function in a
// (one-ring) neighborhood of an input simplex(vertex). This class will allow for us to evaluate how
// a function changes, which might be useful fo ra line-search, but the real purpose is to define an
// interface for a differentiable variant that enables the use of gradient descent.

class LocalFunction : public virtual Function
{
public:
    LocalFunction(std::shared_ptr<PerSimplexFunction> function);
    virtual ~LocalFunction();

public:
    // evaluate the function on the top level simplex of the tuple
    double get_value(const Simplex& simplex) const override;
    const Mesh& mesh() const final override;
    double get_value(const Tuple& simplex) const;
    const PerSimplexFunction& per_simplex_function() const;
    std::shared_ptr<PerSimplexFunction> per_simplex_function_ptr() const;

    PrimitiveType get_simplex_type() const;

protected:
    std::vector<Tuple> get_local_neighborhood_tuples(const Simplex& simplex) const;


private:
    std::shared_ptr<PerSimplexFunction> m_function;
};
} // namespace wmtk::function
