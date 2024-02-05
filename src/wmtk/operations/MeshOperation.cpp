#include "MeshOperation.hpp"

#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

namespace wmtk::operations {
MeshOperation::MeshOperation(Mesh& m)
    : Operation(m)
{}

std::vector<simplex::Simplex> MeshOperation::execute(const simplex::Simplex& simplex)
{
    if (mesh().top_simplex_type() == PrimitiveType::Edge)
        return execute_aux(static_cast<EdgeMesh&>(mesh()), simplex);
    else if (mesh().top_simplex_type() == PrimitiveType::Triangle)
        return execute_aux(static_cast<TriMesh&>(mesh()), simplex);
    else if (mesh().top_simplex_type() == PrimitiveType::Tetrahedron)
        return execute_aux(static_cast<TetMesh&>(mesh()), simplex);
    else
        throw std::runtime_error("invalid mesh type");
}

std::vector<simplex::Simplex> MeshOperation::unmodified_primitives(
    const simplex::Simplex& simplex) const
{
    if (mesh().top_simplex_type() == PrimitiveType::Edge)
        return unmodified_primitives_aux(static_cast<const EdgeMesh&>(mesh()), simplex);
    else if (mesh().top_simplex_type() == PrimitiveType::Triangle)
        return unmodified_primitives_aux(static_cast<const TriMesh&>(mesh()), simplex);
    else if (mesh().top_simplex_type() == PrimitiveType::Tetrahedron)
        return unmodified_primitives_aux(static_cast<const TetMesh&>(mesh()), simplex);
    else
        throw std::runtime_error("invalid mesh type");
}

} // namespace wmtk::operations