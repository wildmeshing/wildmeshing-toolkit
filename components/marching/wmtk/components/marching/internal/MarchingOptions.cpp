#include "MarchingOptions.hpp"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::components {

void to_json(nlohmann::json& j, MarchingOptions& o)
{
    j = {
        {"input", o.input},
        {"output", o.output},
        {"attributes", o.attributes},
        {"input_values", o.input_values},
        {"output_value", o.output_value},
        {"filter_values", o.filter_values},
        {"pass_through", o.pass_through}};
}

void from_json(const nlohmann::json& j, MarchingOptions& o)
{
    o.input = j.at("input");
    o.output = j.at("output");
    o.attributes = j.at("attributes");
    j.at("input_values").get_to(o.input_values);
    o.output_value = j.at("output_value");
    j.at("filter_values").get_to(o.filter_values);
    j.at("pass_through").get_to(o.pass_through);

    if (o.filter_values.size() != o.attributes.filter_labels.size()) {
        log_and_throw_error(
            "Numbers of filters and corresponding values are different. Filters: {}, values: {}.",
            o.attributes.filter_labels.size(),
            o.filter_values.size());
    }
}

} // namespace wmtk::components