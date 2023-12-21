#pragma once
#include <nlohmann/json.hpp>
#include <optional>


namespace wmtk::utils {
class Hashable;
class MerkleTreeInteriorNode;
// returns a json object if there is a difference
std::optional<nlohmann::json>
merkle_tree_diff(const Hashable& a, const Hashable& b, bool detailed = false);


} // namespace wmtk::utils
