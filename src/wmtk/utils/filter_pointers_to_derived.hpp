#pragma once
#include <algorithm>
#include <iterator>
#include <memory>
#include <vector>

namespace wmtk::utils {
// returns the subset of edges that are drived form a particular type in the same order passed in
template <typename Base, typename Derived>
auto filter_pointers_to_derived(const std::vector<std::shared_ptr<Base>>& edges)
    -> std::vector<std::shared_ptr<Derived>>
{
    // dynamic_ptr_cast will convert any invalid conversion to a nullptr so this code
    // * forces every edge to be the derived type, and invalid casts will be set to
    // shared_ptr<Derived>{nullptr}
    // * uses the erase-remove idiom to remove all nullptrs (i.e all elements that oculd not be
    // cast)


    std::vector<std::shared_ptr<Derived>> ret;

    std::transform(
        edges.begin(),
        edges.end(),
        std::back_inserter(ret),
        [](const std::shared_ptr<Base>& ptr) -> std::shared_ptr<Derived> {
            return std::dynamic_pointer_cast<Derived>(ptr);
        });

    ret.erase(std::remove(ret.begin(), ret.end(), std::shared_ptr<Derived>{}), ret.end());
    return ret;
}
} // namespace wmtk::utils
