#pragma once
#include <wmtk/Mesh.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/Simplex.hpp>
#include <wmtk/Tuple.hpp>
namespace wmtk {
namespace function {
class PerSimplexFunction
{
public:
    PerSimplexFunction(const Mesh& mesh, const PrimitiveType& simplex_type);
    virtual ~PerSimplexFunction();

public:
    const Mesh& mesh() const;
    virtual double get_value(const Simplex& s) const = 0;
    const PrimitiveType& get_function_simplex_type() const;

private:
    const Mesh& m_mesh;
    const PrimitiveType m_function_simplex_type;
};
} // namespace function

} // namespace wmtk