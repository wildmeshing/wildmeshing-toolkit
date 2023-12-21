#pragma once
#include <functional>
#include "Hashable.hpp"

namespace wmtk::utils {
class MerkleTreeInteriorNode : public Hashable
{
public:
    std::map<std::string, std::size_t> child_hashes() const override;

    virtual std::map<std::string, const Hashable*> child_hashables() const = 0;
};
} // namespace wmtk::utils
