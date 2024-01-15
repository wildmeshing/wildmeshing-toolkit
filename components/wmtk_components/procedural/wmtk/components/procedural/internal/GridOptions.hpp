#pragma once
#include <variant>

#include "Grid2Options.hpp"
#include "Grid3Options.hpp"



namespace wmtk::components::internal {
class GridOptions
{
    public:
    std::variant<Grid2Options,Grid3Options> opts;
    friend void to_json(nlohmann::json& nlohmann_json_j, const GridOptions& nlohmann_json_t)
    {
        std::visit([&](const auto& o) {
                to_json(nlohmann_json_j, o);
                }, nlohmann_json_t.opts);

    }
    friend void from_json(const nlohmann::json& nlohmann_json_j, GridOptions& nlohmann_json_t) {

        const auto& dimension = nlohmann_json_j["dimensions"];
        assert(dimension.is_array());

        size_t dsize = dimension.size();
        if(dsize == 2) {
            nlohmann_json_t.opts = nlohmann_json_j.get<Grid2Options>();
        } else {
            nlohmann_json_t.opts = nlohmann_json_j.get<Grid3Options>();
        }
    }


};
}
