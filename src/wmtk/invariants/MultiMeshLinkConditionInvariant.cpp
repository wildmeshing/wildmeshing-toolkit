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

namespace wmtk {
namespace {

struct MultiMeshLinkConditionFunctor
{
    bool operator()(const Mesh& m, const simplex::Simplex& s) const { return true; }
    bool operator()(const PointMesh& m, const simplex::Simplex& s) const { return true; }

    bool operator()(const EdgeMesh& m, const simplex::Simplex& s) const
    {
        return simplex::link_condition(m, s.tuple());
    }
    bool operator()(const TriMesh& m, const simplex::Simplex& s) const
    {
        return simplex::link_condition(m, s.tuple());
    }
    bool operator()(const TetMesh& m, const simplex::Simplex& s) const
    {
        return simplex::link_condition(m, s.tuple());
    }
};
} // namespace

MultiMeshLinkConditionInvariant::MultiMeshLinkConditionInvariant(const Mesh& m)
    : Invariant(m)
{}
bool MultiMeshLinkConditionInvariant::before(const simplex::Simplex& t) const
{
    assert(t.primitive_type() == PrimitiveType::Edge);
    multimesh::MultiMeshSimplexVisitor visitor(
        std::integral_constant<int64_t, 1>{}, // specify that this runs on edges
        MultiMeshLinkConditionFunctor{});
    // TODO: fix visitor to work for const data
    visitor.execute_from_root(const_cast<Mesh&>(mesh()), t);
    const auto& data = visitor.cache();

    for (const auto& [key, value_var] : data) {
        const bool valid = std::get<bool>(value_var);
        if (!valid) {
            return false;
        }
    }

    // auto m_multimesh_manager = mesh().multimesh_manager();
    // for (int64_t child_id = 0; child_id < m_multimesh_manager.child_meshes.siz(); child_id++) {
    //     auto child_mesh_va_tuples =
    //         m_multimesh_manager.map_to_child_tuples(m, child_id,
    //         simplex::Simplex(PrimitiveType::Vertex, t));
    //     auto child_mesh_vb_tuples = m_multimesh_manager.map_to_child_tuples(
    //         m,
    //         child_id,
    //         simplex::Simplex(PrimitiveType::Vertex, mesh().switch_tuple(t,
    //         PrimitiveType::Vertex)));
    //     auto child_mesh_eab_tuples =
    //         m_multimesh_manager.map_to_child_tuples(m, child_id,
    //         simplex::Simplex(PrimitiveType::Edge, t));

    //     if (!child_mesh_va_tuples.empty() && !child_mesh_vb_tuples.empty() &&
    //         child_mesh_eab_tuples.empty()) {
    //         return false;
    //     }
    // }
    return true;
}
} // namespace wmtk
