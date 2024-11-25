#pragma once
#include "attribute_error.hpp"
namespace wmtk::components::multimesh::utils {
struct AttributeDescription;
}

namespace wmtk::components::multimesh::utils::detail {

class attribute_ambiguous_error : public attribute_error
{
public:
    static std::string make_message(
        const AttributeDescription& description,
        const std::vector<AttributeDescription>& possibilities);
    attribute_ambiguous_error(
        const AttributeDescription& d,
        const std::vector<AttributeDescription>& possibilities)
        : attribute_error(make_message(d, possibilities), d)
    {}
    template <typename... Args>
    static attribute_ambiguous_error make(
        const std::vector<AttributeDescription>& possiblities,
        Args&&... args)
    {
        return attribute_ambiguous_error(
            AttributeDescription{std::forward<Args>(args)...},
            possiblities);
    }
};


} // namespace wmtk::components::multimesh::utils::detail
