#pragma once
#include <wmtk/multimesh/MultiMeshManager.hpp>

namespace wmtk::tests {

class DEBUG_MultiMeshManager : public multimesh::MultiMeshManager
{
public:
    using multimesh::MultiMeshManager::child_to_parent_map_attribute_name;
    using multimesh::MultiMeshManager::children;
    using multimesh::MultiMeshManager::is_root;
    using multimesh::MultiMeshManager::parent_to_child_map_attribute_name;

    static void run_checks(const Mesh& m);

    // these run catch2 tests
    void check_map_valid(const Mesh& my_mesh) const;
    void check_child_mesh_valid(const Mesh& my_mesh, const Mesh& child_mesh) const;
    void check_child_map_valid(const Mesh& my_mesh, const ChildData& child_data) const;
};

} // namespace wmtk::tests
