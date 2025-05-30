#include "MultiMeshMapValidInvariant.hpp"

#include <stdexcept>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/cofaces_single_dimension_iterable.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include "wmtk/simplex/cofaces_single_dimension.hpp"

namespace wmtk {

MultiMeshMapValidInvariant::MultiMeshMapValidInvariant(const Mesh& m)
    : Invariant(m, true, false, false)
{}
bool MultiMeshMapValidInvariant::before(const simplex::Simplex& t) const
{
    throw("removed due to removal of multimesh");
    return true;
}
} // namespace wmtk
