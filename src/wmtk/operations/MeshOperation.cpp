#include "MeshOperation.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::operations {
MeshOperation::MeshOperation(Mesh& m)
    : Operation(m)
{}

std::vector<Simplex> MeshOperation::execute(const Simplex& simplex)
{
    if (mesh().top_simplex_type() == PrimitiveType::Edge)
        return execute(static_cast<EdgeMesh&>(mesh()), simplex);
    else if (mesh().top_simplex_type() == PrimitiveType::Face)
        return execute(static_cast<TriMesh&>(mesh()), simplex);
    else if (mesh().top_simplex_type() == PrimitiveType::Tetrahedron)
        return execute(static_cast<TetMesh&>(mesh()), simplex);
    else
        throw std::runtime_error("invalid mesh type");
}

std::vector<Simplex> MeshOperation::unmodified_primitives(const Simplex& simplex) const
{
    if (mesh().top_simplex_type() == PrimitiveType::Edge)
        return unmodified_primitives(static_cast<const EdgeMesh&>(mesh()), simplex);
    else if (mesh().top_simplex_type() == PrimitiveType::Face)
        return unmodified_primitives(static_cast<const TriMesh&>(mesh()), simplex);
    else if (mesh().top_simplex_type() == PrimitiveType::Tetrahedron)
        return unmodified_primitives(static_cast<const TetMesh&>(mesh()), simplex);
    else
        throw std::runtime_error("invalid mesh type");
}

} // namespace wmtk::operations