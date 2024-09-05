#pragma once

#include <nlohmann/json.hpp>

namespace wmtk::components {

struct MultimeshUVOptions
{
public:
    std::string type;
    std::string parent;
    std::string child;
    std::string name;
};

struct MultimeshBOptions
{
public:
    std::string type;
    std::string name;
    std::string mesh;
    std::string tag_name;
    nlohmann::json position;
};

struct MultimeshTOptions
{
public:
    std::string type;
    std::string name;
    std::string mesh;
    nlohmann::json position;
    std::string tag;
    int64_t tag_value;
    int64_t primitive;
};

NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultimeshUVOptions, type, parent, child, name);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MultimeshBOptions, type, name, mesh, position, tag_name);
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(
    MultimeshTOptions,
    type,
    name,
    mesh,
    position,
    tag,
    tag_value,
    primitive);

} // namespace wmtk::components
