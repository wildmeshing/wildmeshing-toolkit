#include <fmt/format.h>
#include <wmtk/attribute/internal/hash.hpp>
#include "Mesh.hpp"
namespace wmtk {
std::map<std::string, const wmtk::utils::Hashable*> Mesh::child_hashables() const
{
    std::map<std::string, const wmtk::utils::Hashable*> ret;
    ret["attribute_manager"] = &m_attribute_manager;
    ret["multimesh_manager"] = &m_multi_mesh_manager;
    return ret;
}
std::map<std::string, std::size_t> Mesh::child_hashes() const
{
    // default implementation pulls the child attributes (ie the attributes)
    std::map<std::string, std::size_t> ret = wmtk::utils::MerkleTreeInteriorNode::child_hashes();


    const std::hash<TypedAttributeHandle<char>> cattr_hasher;
    const std::hash<TypedAttributeHandle<int64_t>> attr_hasher;
    for (size_t j = 0; j < m_flag_handles.size(); ++j) {
        ret[fmt::format("flag_handle_{}", j)] = cattr_hasher(m_flag_handles[j]);
    }

    return ret;
}
} // namespace wmtk
