#include "MeshConsolidate.hpp"
#include <wmtk/multimesh/consolidate.hpp>


namespace wmtk::operations {
MeshConsolidate::MeshConsolidate(Mesh& m)
    : Operation(m)
{
    operation_name = "MeshConsolidate";
}

std::vector<simplex::Simplex> MeshConsolidate::unmodified_primitives(
    const simplex::Simplex& simplex) const
{
    return {simplex};
}

std::vector<simplex::Simplex> MeshConsolidate::execute(const simplex::Simplex& simplex)
{
    multimesh::consolidate(mesh());

    return {simplex};
}

bool MeshConsolidate::before(const simplex::Simplex& simplex) const
{
    return true;
}

bool MeshConsolidate::after(
    const std::vector<simplex::Simplex>& unmods,
    const std::vector<simplex::Simplex>& mods) const
{
    return true;
}

} // namespace wmtk::operations