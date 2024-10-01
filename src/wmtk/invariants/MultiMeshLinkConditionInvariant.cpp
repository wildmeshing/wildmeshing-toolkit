#include "MultiMeshLinkConditionInvariant.hpp"

#include <stdexcept>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/multimesh/MultiMeshSimplexVisitor.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/link_condition.hpp>

namespace wmtk::invariants {
namespace {

struct MultiMeshLinkConditionFunctor
{
    bool operator()(const auto& m, const simplex::Simplex& s, int64_t) {
        return this->operator()(m,s);
    }
    template <typename MeshType>
    bool operator()(const MeshType& m, const simplex::Simplex& s) const
    {
        return m.is_free() || simplex::link_condition(m, s.tuple());
    }
};
} // namespace

MultiMeshLinkConditionInvariant::MultiMeshLinkConditionInvariant(const Mesh& m)
    : Invariant(m, true, false, false)
{}
bool MultiMeshLinkConditionInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == PrimitiveType::Edge);
    multimesh::MultiMeshSimplexVisitor visitor(
        std::integral_constant<int64_t, 1>{}, // specify that this runs on edges
        MultiMeshLinkConditionFunctor{});
    // TODO: fix visitor to work for const data
    visitor.execute_from_root(const_cast<Mesh&>(mesh()), simplex::NavigatableSimplex(mesh(),t));
    const auto& data = visitor.cache();

    for (const auto& [key, value_var] : data) {
        const bool valid = std::get<bool>(value_var);
        if (!valid) {
            return false;
        }
    }

    return true;
}
} // namespace wmtk::invariants
