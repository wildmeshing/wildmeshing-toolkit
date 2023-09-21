#pragma once

namespace wmtk {

template <typename InvariantType>
bool find_invariant_in_collection_by_type(const InvariantCollection& invariants)
{
    // at least one invariant must exist if we're checking for them
    if (invariants.empty()) {
        return false;
    }
    for (const auto& op : invariants.invariants()) {
        if (dynamic_cast<const InvariantType*>(op.get()) != nullptr) {
            return true;
        }
    }
    return false;
}

template <typename... InvariantTypes>

bool find_invariants_in_collection_by_type(const InvariantCollection& invariants)
{
    if (invariants.size() < sizeof...(InvariantTypes)) {
        return false;
    }
    return (find_invariant_in_collection_by_type<InvariantTypes>(invariants) && ... && true);
}

} // namespace wmtk
