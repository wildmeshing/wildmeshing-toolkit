#include "RegularSpace.hpp"

#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapseToMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitAtMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSwap.hpp>
#include <wmtk/operations/tri_mesh/VertexTangentialLaplacianSmooth.hpp>

namespace wmtk::components::internal {

RegularSpace::RegularSpace(TriMesh& mesh, const bool lock_boundary)
    : m_mesh{mesh}
    , m_lock_boundary{lock_boundary}
    , m_scheduler(m_mesh)
{}

void RegularSpace::process(const long iterations) {}

} // namespace wmtk::components::internal
