#include "MultiMeshTopologyInvariant.hpp"
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>

namespace wmtk {
MultiMeshTopologyInvariant::MultiMeshTopologyInvariant(const Mesh& m)
    : MeshInvariant(m)
{}

bool MultiMeshTopologyInvariant::before(const Tuple& t)
{
    // TODO: use multimesh visitor
    return true;
}
} // namespace wmtk