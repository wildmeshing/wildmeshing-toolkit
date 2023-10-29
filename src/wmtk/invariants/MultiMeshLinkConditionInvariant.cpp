#include "MultiMeshLinkConditionInvariant.hpp"
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
namespace {

struct MultiMeshLinkConditionFunctor
{
    bool operator()(const Mesh& m, const simplex::Simplex& s) const { return true; }
    bool operator()(const PointMesh& m, const simplex::Simplex& s) const { return true; }
    bool operator()(const EdgeMesh& m, const simplex::Simplex& s) const { return true; }

    bool operator()(const TriMesh& m, const simplex::Simplex& s) const
    {
        return SimplicialComplex::link_cond_bd_2d(m, s.tuple());
    }
    bool operator()(const TetMesh& m, const simplex::Simplex& s) const
    {
        throw std::runtime_error("implement link condition in multimesh trimesh");
        return false;
    }
};
} // namespace

MultiMeshLinkConditionInvariant::MultiMeshLinkConditionInvariant(const Mesh& m)
    : MeshInvariant(m)
{}
bool MultiMeshLinkConditionInvariant::before(const Tuple& t) const
{
    multimesh::MultiMeshVisitor visitor(
        std::integral_constant<long, 1>{}, // specify that this runs on edges
        MultiMeshLinkConditionFunctor{});
    // TODO: fix visitor to work for const data
    auto data =
        visitor.execute_from_root(const_cast<Mesh&>(mesh()), Simplex(PrimitiveType::Edge, t));

    for (const auto& [key, value_var] : data) {
        const bool valid = std::get<bool>(value_var);
        if (!valid) {
            return false;
        }
    }
    return true;
}
} // namespace wmtk

