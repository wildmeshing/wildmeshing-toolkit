
#include "merkle_tree.hpp"
#include "Hashable.hpp"
#include "MerkleTreeInteriorNode.hpp"
namespace wmtk::utils {
nlohmann::json merkle_tree(const Hashable& hashable)
{
    nlohmann::json js;
    for (const auto& [name, hash] : hashable.child_hashes()) {
        js[name] = hash;
    }

    if (auto merkle_ptr = dynamic_cast<const MerkleTreeInteriorNode*>(&hashable);
        merkle_ptr != nullptr) {
        for (const auto& [name, hashable_ptr] : merkle_ptr->child_hashables()) {
            js[name] = merkle_tree(*hashable_ptr);
        }
    }
    return js;
}
} // namespace wmtk::utils
