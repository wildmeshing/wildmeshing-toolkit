#include "RegularSpaceOptions.hpp"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::internal {

void to_json(nlohmann::json& j, RegularSpaceOptions& o)
{
    j = {
        {"input", o.input},
        {"output", o.output},
        {"attributes", o.attributes},
        {"values", o.values},
        {"pass_through", o.pass_through}};
}

void from_json(const nlohmann::json& j, RegularSpaceOptions& o)
{
    o.input = j.at("input");
    o.output = j.at("output");
    o.attributes = j.at("attributes");
    j.at("values").get_to(o.values);
    j.at("pass_through").get_to(o.pass_through);

    if (o.attributes.size() != o.values.size()) {
        log_and_throw_error(
            "One value must be given for each attribute. Attributes: {}, values{}.",
            o.attributes.size(),
            o.values.size());
    }
}

} // namespace wmtk::components::internal