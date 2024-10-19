#pragma once

#include <wmtk/components/utils/json_utils.hpp>

#include <nlohmann/json.hpp>
#include "DiskOptions.hpp"
#include "GridOptions.hpp"
#include "TriangleFanOptions.hpp"

namespace wmtk::components::procedural {


struct ProceduralOptions
{
    std::string name;
    std::variant<GridOptions, TriangleFanOptions, DiskOptions> settings;

    friend void to_json(nlohmann::json& nlohmann_json_j, const ProceduralOptions& nlohmann_json_t);
    friend void from_json(const nlohmann::json& nlohmann_json_j, ProceduralOptions& nlohmann_json_t);
};


} // namespace wmtk::components::procedural
