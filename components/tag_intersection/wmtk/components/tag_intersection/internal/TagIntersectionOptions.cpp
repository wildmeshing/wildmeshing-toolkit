#include "TagIntersectionOptions.hpp"

#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::internal {

void to_json(nlohmann::json& j, TagIntersectionOptions& o)
{
    j = {
        {"input", o.input},
        {"output", o.output},
        {"attributes", o.attributes},
        {"values", o.values},
        {"output_attributes", o.output_attributes},
        {"output_values", o.output_values},
        {"pass_through", o.pass_through}};
}

void from_json(const nlohmann::json& j, TagIntersectionOptions& o)
{
    o.input = j.at("input");
    o.output = j.at("output");
    o.attributes = j.at("attributes");
    j.at("values").get_to(o.values);
    o.output_attributes = j.at("output_attributes");
    j.at("output_values").get_to(o.output_values);
    j.at("pass_through").get_to(o.pass_through);

    if ((o.attributes.vertex_labels.size() != o.values.vertex_values.size()) ||
        (o.attributes.edge_labels.size() != o.values.edge_values.size()) ||
        (o.attributes.face_labels.size() != o.values.face_values.size()) ||
        (o.attributes.tetrahedron_labels.size() != o.values.tetrahedron_values.size())) {
        log_and_throw_error(
            "One value must be given for each attribute.\n  vertex_labels: {}, vertex_values: {}\n "
            " edge_labels: {}, edge_values: {}\n  face_labels: {}, face_values: {}\n  "
            "tetrahedron_labels: {}, tetrahedron_values: {}",
            o.attributes.vertex_labels.size(),
            o.values.vertex_values.size(),
            o.attributes.edge_labels.size(),
            o.values.edge_values.size(),
            o.attributes.face_labels.size(),
            o.values.face_values.size(),
            o.attributes.tetrahedron_labels.size(),
            o.values.tetrahedron_values.size());
    }
    if ((o.output_attributes.vertex_labels.size() != o.output_values.vertex_values.size()) ||
        (o.output_attributes.edge_labels.size() != o.output_values.edge_values.size()) ||
        (o.output_attributes.face_labels.size() != o.output_values.face_values.size()) ||
        (o.output_attributes.tetrahedron_labels.size() !=
         o.output_values.tetrahedron_values.size())) {
        log_and_throw_error(
            "One output value must be given for each output attribute.\n  vertex_labels: {}, "
            "vertex_values: {}\n edge_labels: {}, edge_values: {}\n  face_labels: {}, face_values: "
            "{}\n  tetrahedron_labels: {}, tetrahedron_values: {}",
            o.output_attributes.vertex_labels.size(),
            o.output_values.vertex_values.size(),
            o.output_attributes.edge_labels.size(),
            o.output_values.edge_values.size(),
            o.output_attributes.face_labels.size(),
            o.output_values.face_values.size(),
            o.output_attributes.tetrahedron_labels.size(),
            o.output_values.tetrahedron_values.size());
    }
}

} // namespace wmtk::components::internal