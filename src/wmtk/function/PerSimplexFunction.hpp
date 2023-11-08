#pragma once
#include <wmtk/Mesh.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/Simplex.hpp>
#include <wmtk/Tuple.hpp>
#include <wmtk/attribute/AttributeHandle.hpp>
namespace wmtk::function {
class PerSimplexFunction
{
public:
    PerSimplexFunction(const Mesh& mesh, const PrimitiveType& domain_simplex_type);
    virtual ~PerSimplexFunction();

public:
    const Mesh& mesh() const;

    /**
     * @brief the function is defined over the domain that's represented by the argument
     *
     * @param simplex
     * @return double
     */
    virtual double get_value(const simplex::Simplex& domain_simplex) const = 0;
    // helper because in many cases we want to compute the value of multiple simplices at once
    double get_value_sum(const std::vector<Simplex>& domain_simplices) const;

    // get domain simplex_type
    PrimitiveType get_domain_simplex_type() const;

private:
    const Mesh& m_mesh;
    const PrimitiveType m_domain_simplex_type;
};
} // namespace wmtk::function
