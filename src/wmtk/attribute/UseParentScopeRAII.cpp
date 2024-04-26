// clang-format off
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/TetMesh.hpp>
// clang-format on
#if defined(WMTK_ENABLE_MULTIMESH)
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
#endif

namespace wmtk::attribute {

UseParentScopeRAII::UseParentScopeRAII(Mesh& mesh)
    : m_mesh(mesh)
{
#if defined(WMTK_ENABLE_MULTIMESH)
    auto run = [](auto&& m) {
        if constexpr (!std::is_const_v<std::decay_t<decltype(m)>>) {
            m.m_attribute_manager.change_to_parent_scope();
        }
    };
    multimesh::MultiMeshVisitor visitor(run);
    visitor.execute_from_root(m_mesh);
#else
    m_mesh.m_attribute_manager.change_to_parent_scope();
#endif
}
UseParentScopeRAII::~UseParentScopeRAII()
{
#if defined(WMTK_ENABLE_MULTIMESH)
    auto run = [](auto&& m) {
        if constexpr (!std::is_const_v<std::decay_t<decltype(m)>>) {
            m.m_attribute_manager.change_to_child_scope();
        }
    };
    multimesh::MultiMeshVisitor visitor(run);
    visitor.execute_from_root(m_mesh);
#else
    m_mesh.m_attribute_manager.change_to_child_scope();
#endif
}
} // namespace wmtk::attribute
