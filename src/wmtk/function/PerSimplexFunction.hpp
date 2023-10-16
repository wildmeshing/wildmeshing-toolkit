#pragma once
#include <wmtk/Mesh.hpp>
#include <wmtk/MeshAttributeHandle.hpp>
#include <wmtk/Simplex.hpp>
#include <wmtk/Tuple.hpp>
namespace wmtk {
namespace function {
class PerSimplexFunction
{
public:
    PerSimplexFunction(const Mesh& mesh, const Simplex::Type& simplex_type);
    virtual ~PerSimplexFunction();

    const Mesh& mesh() const;

public:
    virtual double get_value(const Simplex& s) const = 0;

    const Simplex::Type& get_simplex_type() const;

private:
    const Mesh& m_mesh;
    const Simplex::Type m_simplex_type;
};
} // namespace function

} // namespace wmtk