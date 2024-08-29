#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {
struct TriInsOptions
{
    std::string input;
    std::string input_position;
    std::string background;
    std::string background_position;
    std::string name;
    bool make_child_free;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    TriInsOptions,
    input,
    input_position,
    background,
    background_position,
    name,
    make_child_free
    );

} // namespace wmtk::components
