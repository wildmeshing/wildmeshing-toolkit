// clang-format off
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/TetMesh.hpp>
// clang-format on
#include <wmtk/multimesh/MultiMeshVisitor.hpp>

namespace wmtk::multimesh::attribute {

UseParentScopeRAII::UseParentScopeRAII(Mesh& mesh)
    : m_mesh(mesh)
{
    auto run = [](auto&& m) {
        if constexpr (!std::is_const_v<std::decay_t<decltype(m)>>) {
            m.m_attribute_manager.change_to_parent_scope();
        }
    };
    MultiMeshVisitor visitor(run);
    visitor.execute_from_root(m_mesh);
}
UseParentScopeRAII::~UseParentScopeRAII()
{
    auto run = [](auto&& m) {
        if constexpr (!std::is_const_v<std::decay_t<decltype(m)>>) {
            m.m_attribute_manager.change_to_leaf_scope();
        }
    };
    MultiMeshVisitor visitor(run);
    visitor.execute_from_root(m_mesh);
}
} // namespace wmtk::multimesh::attribute
