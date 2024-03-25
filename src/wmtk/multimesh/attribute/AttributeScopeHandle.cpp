#include "AttributeScopeHandle.hpp"
#include <wmtk/EdgeMesh.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/attribute/AttributeScopeHandle.hpp>
#include <wmtk/multimesh/MultiMeshVisitor.hpp>
namespace wmtk::multimesh::attribute {
AttributeScopeHandle::AttributeScopeHandle(Mesh& m)
{
    auto get_handles = [&](auto&& mesh) {
        using T = std::remove_reference_t<decltype(mesh)>;
        if constexpr (!std::is_const_v<T>) {
            m_scopes.emplace_back(mesh.create_single_mesh_scope());
        }
    };

    MultiMeshVisitor(get_handles).execute_from_root(m);
}


void AttributeScopeHandle::mark_failed()
{
    for (single_handle_type& scope : m_scopes) {
        scope.mark_failed();
    }
}
} // namespace wmtk::multimesh::attribute
