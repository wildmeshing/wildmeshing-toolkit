#include "IsotropicRemeshingOptions.hpp"
#include <fmt/format.h>
#include <algorithm>
#include <nlohmann/json.hpp>

#define DEFAULT_PARSABLE_ARGS                                                                \
    iterations, length_abs, length_rel, lock_boundary, use_for_periodic, dont_disable_split, \
        fix_uv_seam, envelope_size

namespace wmtk::components::isotropic_remeshing {

void to_json(nlohmann::json& nlohmann_json_j, const IsotropicRemeshingOptions& nlohmann_json_t)
{
    NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(NLOHMANN_JSON_TO, DEFAULT_PARSABLE_ARGS));
    {
        std::string name;
        switch (nlohmann_json_t.edge_swap_mode) {
        case EdgeSwapMode::AMIPS: name = "amips"; break;
        case EdgeSwapMode::Valence: name = "valence"; break;
        default:
        case EdgeSwapMode::Skip: name = "skip"; break;
        }
        nlohmann_json_j["edge_swap_mode"] = name;
    }
}
void from_json(const nlohmann::json& nlohmann_json_j, IsotropicRemeshingOptions& nlohmann_json_t)
{
    NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(
        NLOHMANN_JSON_FROM,
        DEFAULT_PARSABLE_ARGS

        ));
    assert(nlohmann_json_j.contains("edge_swap_mode"));
    const std::string swap_name = nlohmann_json_j["edge_swap_mode"].get<std::string>();
    if (swap_name == "amips") {
        nlohmann_json_t.edge_swap_mode = EdgeSwapMode::AMIPS;
    } else if (swap_name == "valence") {
        nlohmann_json_t.edge_swap_mode = EdgeSwapMode::Valence;
    } else if (swap_name == "skip") {
        nlohmann_json_t.edge_swap_mode = EdgeSwapMode::Skip;
    } else {
        throw std::runtime_error(fmt::format(
            "Expected edge_swap_mode to be one of [amips,valence,skip], got [{}]",
            swap_name));
    }
}
void IsotropicRemeshingOptions::load_json(const nlohmann::json& js)
{
    from_json(js, *this);
}
void IsotropicRemeshingOptions::write_json(nlohmann::json& js) const
{
    to_json(js, *this);
}

std::vector<wmtk::attribute::MeshAttributeHandle> IsotropicRemeshingOptions::all_positions() const
{
    auto r = other_position_attributes;
    r.emplace_back(position_attribute);
    if (inversion_position_attribute.has_value()) {
        r.emplace_back(inversion_position_attribute.value());
    }
    std::sort(r.begin(), r.end());

    r.erase(std::unique(r.begin(), r.end()), r.end());

    return r;
}

} // namespace wmtk::components::isotropic_remeshing
