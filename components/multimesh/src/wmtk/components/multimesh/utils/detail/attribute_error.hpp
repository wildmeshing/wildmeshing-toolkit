#pragma once
#include <optional>
#include <stdexcept>
#include <wmtk/Primitive.hpp>
#include <wmtk/attribute/AttributeType.hpp>
#include "../AttributeDescription.hpp"

namespace wmtk::components::multimesh::utils::detail {

class attribute_error : public std::range_error
{
public:
    template <typename... Args>
    attribute_error(const std::string_view& message, Args&&... args)
        : std::range_error(std::string(message))
        , description(std::forward<Args>(args)...)
    {}
    AttributeDescription description;
};
} // namespace wmtk::components::multimesh::utils::detail
