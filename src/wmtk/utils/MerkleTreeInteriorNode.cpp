

#include "MerkleTreeInteriorNode.hpp"

namespace wmtk::utils {

std::map<std::string, std::size_t> MerkleTreeInteriorNode::child_hashes() const
{
    std::map<std::string, std::size_t> child_hashes;
    for (const auto& [k, v] : child_hashables()) {
        child_hashes[k] = v->hash();
    }
    return child_hashes;
}
} // namespace wmtk::utils
