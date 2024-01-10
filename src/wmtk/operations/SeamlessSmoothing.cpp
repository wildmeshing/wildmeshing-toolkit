#include "SeamlessSmoothing.hpp"

#include <polysolve/nonlinear/Problem.hpp>
#include <polysolve/nonlinear/Solver.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/MutableAccessor.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/SeamlessConstraints.hpp>

namespace wmtk::operations {
// ok
SeamlessSmoothing::SeamlessSmoothing(
    TriMesh& ref_mesh,
    TriMesh& cut_mesh,
    std::shared_ptr<wmtk::function::Function> energy)
    : OptimizationSmoothing(ref_mesh, energy)
    , m_cut_mesh(cut_mesh)
    , m_ref_mesh(ref_mesh)
{}

std::vector<simplex::Simplex> SeamlessSmoothing::execute(const simplex::Simplex& simplex)
{
    // map simplex to cut_mesh
    std::vector<simplex::Simplex> vs_on_cut_mesh = mesh().map_to_child(m_cut_mesh, simplex);
    if (m_cut_mesh.is_boundary(vs_on_cut_mesh[0])) {
        // TODO: Implement the quesio
        return {};
    } else {
        mesh() = m_cut_mesh;
        OptimizationSmoothing::execute(vs_on_cut_mesh[0]);
        mesh() = m_ref_mesh;
    }


    return AttributesUpdate::execute(simplex);
}

} // namespace wmtk::operations
