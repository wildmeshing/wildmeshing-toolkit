#pragma once
#include "attribute_error.hpp"
namespace wmtk::components::multimesh::utils {
struct AttributeDescription;
}

namespace wmtk::components::multimesh::utils::detail {

class attribute_missing_error : public attribute_error
{
public:
    static std::string make_message(const AttributeDescription& description);
    attribute_missing_error(const AttributeDescription& d)
        : attribute_error(make_message(d), d)
    {}
    template <typename... Args>
    static attribute_missing_error make(Args&&... args)
    {
        return attribute_missing_error(AttributeDescription{std::forward<Args>(args)...});
    }
};


} // namespace wmtk::components::multimesh::utils::detail
