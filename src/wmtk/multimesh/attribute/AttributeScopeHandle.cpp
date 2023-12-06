#include "AttributeScopeHandle.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/attribute/AttributeScopeHandle.hpp>
namespace wmtk::multimesh::attribute {
AttributeScopeHandle::AttributeScopeHandle(Mesh& m) {}

AttributeScopeHandle::~AttributeScopeHandle() = default;

void AttributeScopeHandle::mark_failed()
{
    for (single_scope_handle& scope : m_scopes) {
        scope.mark_failed();
    }
}
} // namespace wmtk::multimesh::attribute
