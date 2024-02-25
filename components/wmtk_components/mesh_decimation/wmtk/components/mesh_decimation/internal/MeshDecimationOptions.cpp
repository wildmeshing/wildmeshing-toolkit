#include "MeshDecimationOptions.hpp"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::internal {

void to_json(nlohmann::json& j, MeshDecimationOptions& o)
{
    j = {
        {"input", o.input},
        {"output", o.output},
        {"constrait_value", o.constrait_value},
        {"decimation_factor", o.decimation_factor},
        {"tet_constrait_tag_name", o.tet_constrait_tag_name},
        {"pass_through", o.pass_through}};
}

void from_json(const nlohmann::json& j, MeshDecimationOptions& o)
{
    o.input = j.at("input");
    o.output = j.at("output");
    o.constrait_value = j.at("constrait_value");
    o.decimation_factor = j.at("decimation_factor");
    o.tet_constrait_tag_name = j.at("tet_constrait_tag_name");
    j.at("pass_through").get_to(o.pass_through);
}

} // namespace wmtk::components::internal