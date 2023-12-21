#pragma once
#include <nlohmann/json.hpp>


namespace wmtk::utils {
class Hashable;
nlohmann::json merkle_tree(const Hashable& m);
} // namespace wmtk::utils
