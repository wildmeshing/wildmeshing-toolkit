#include <wmtk/Mesh.hpp>

namespace wmtk::attribute {

UseParentScopeRAII::UseParentScopeRAII(Mesh& mesh)
    : m_mesh(mesh)
{
    m_mesh.m_attribute_manager.change_to_parent_scope();
}
UseParentScopeRAII::~UseParentScopeRAII()
{
    m_mesh.m_attribute_manager.change_to_child_scope();
}
} // namespace wmtk::attribute
