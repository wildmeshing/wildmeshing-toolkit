#include <wmtk/MultiMeshManager.hpp>

namespace wmtk::tests {

class DEBUG_MultiMeshManager : public MultiMeshManager
{
public:
    using MultiMeshManager::child_to_parent_map_attribute_name;
    using MultiMeshManager::children;
    using MultiMeshManager::is_root;
    using MultiMeshManager::parent_to_child_map_attribute_name;
};
} // namespace wmtk::tests
