#pragma once
#include <wmtk/Mesh.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/Simplex.hpp>
#include <wmtk/Tuple.hpp>
#include "Function.hpp"
namespace wmtk::function {
class PerSimplexFunction : public virtual Function
{
public:
    PerSimplexFunction(const Mesh& mesh, const PrimitiveType& simplex_type);
    virtual ~PerSimplexFunction();

public:
    using Function::get_value;
    const Mesh& mesh() const final override;
    virtual double get_value(const Tuple& s) const = 0;
    double get_value(const simplex::Simplex& s) const final override;
    // the type of simplex that this function operates on
    PrimitiveType get_simplex_type() const;

    // helper because in many cases we want to compute the value of multiple simplices at once
    double get_value_sum(const std::vector<Simplex>& simplices) const;
    // assumes that the underlying simplices are all of the same as get_simplex_type()
    double get_value_sum(const std::vector<Tuple>& tuples) const;

private:
    const Mesh& m_mesh;
    const PrimitiveType m_simplex_type;
};

} // namespace wmtk::function
