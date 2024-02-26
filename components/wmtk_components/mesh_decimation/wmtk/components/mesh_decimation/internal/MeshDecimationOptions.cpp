#include "MeshDecimationOptions.hpp"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::internal {

void to_json(nlohmann::json& j, MeshDecimationOptions& o)
{
    j = {
        {"input", o.input},
        {"output", o.output},
        {"target_len", o.target_len},
        {"cell_constrait_tag_name", o.cell_constrait_tag_name},
        {"attributes", o.attributes},
        {"pass_through", o.pass_through}};
}

void from_json(const nlohmann::json& j, MeshDecimationOptions& o)
{
    o.input = j.at("input");
    o.output = j.at("output");
    o.target_len = j.at("target_len");
    o.cell_constrait_tag_name = j.at("cell_constrait_tag_name");
    j.at("attributes").get_to(o.attributes);
    j.at("pass_through").get_to(o.pass_through);
}

} // namespace wmtk::components::internal