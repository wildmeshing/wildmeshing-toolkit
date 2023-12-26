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
     * @brief This function is defined over a simplex (normally a triangle or tetrahedron). And the
     * domain of the function is represented by the input argument domain_simplex.
     *
     * @param domain_simplex The domain that the function is defined over.
     * @return double The numerical value of the function at the input domain.
     */
    virtual double get_value(const simplex::Simplex& domain_simplex) const = 0;

    // helper because in many cases we want to compute the value of multiple simplices at once
    double get_value_sum(const std::vector<Simplex>& domain_simplices) const;

    // helper because in many cases we want to compute the value of multiple simplices at once
    double get_value_sum(const std::vector<Tuple>& domain_simplices) const;

    // For ExtremeOpt, we might need to Optimize the E_max value
    double get_value_max(const std::vector<Simplex>& domain_simplices) const;
    double get_value_max(const std::vector<Tuple>& domain_simplices) const;

    // Get all values for debugging
    std::vector<double> get_value_all(const std::vector<Simplex>& domain_simplices) const;

    // get domain simplex_type
    PrimitiveType get_domain_simplex_type() const;

protected:
    Simplex as_domain_simplex(const Tuple& t) const;

private:
    const Mesh& m_mesh;
    const PrimitiveType m_domain_simplex_type;
};
} // namespace wmtk::function
