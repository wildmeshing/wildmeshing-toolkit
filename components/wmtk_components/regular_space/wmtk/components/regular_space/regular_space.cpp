#include "regular_space.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/io/Cache.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/primitive_range.hpp>

#include "internal/RegularSpace.hpp"
#include "internal/RegularSpaceOptions.hpp"

namespace wmtk::components {

void regular_space(const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    RegularSpaceOptions options = j.get<RegularSpaceOptions>();

    // input
    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);

    Mesh& mesh = static_cast<Mesh&>(*mesh_in);

    // collect labels that were there already before this component
    std::vector<attribute::MeshAttributeHandle> original_attributes;

    std::vector<attribute::MeshAttributeHandle> label_attributes;
    for (const PrimitiveType& ptype : utils::primitive_below(mesh.top_simplex_type())) {
        std::string attr_name;
        switch (ptype) {
        case PrimitiveType::Vertex: {
            attr_name = "vertex_label";
            break;
        }
        case PrimitiveType::Edge: {
            attr_name = "edge_label";
            break;
        }
        case PrimitiveType::Face: {
            attr_name = "face_label";
            break;
        }
        case PrimitiveType::Tetrahedron: {
            attr_name = "tetrahedron_label";
            break;
        }
        default: log_and_throw_error("Unknown primitive type: {}", ptype);
        }

        if (options.attributes.find(attr_name) == options.attributes.end()) {
            // no attribute was given for that primitive type
            auto h = mesh.register_attribute<int64_t>(attr_name + "_regular_space", ptype, 1);
            label_attributes.emplace_back(h);
        } else {
            auto h = mesh.get_attribute_handle<int64_t>(options.attributes.at(attr_name), ptype);
            label_attributes.emplace_back(h);
            original_attributes.emplace_back(h);
        }
    }

    auto pass_through_attributes = base::get_attributes(mesh, options.pass_through);


    RegularSpace rs(mesh, label_attributes, options.values, pass_through_attributes);
    rs.regularize_tags();

    if (original_attributes.size() != label_attributes.size()) {
        // clean up attributes
        std::vector<attribute::MeshAttributeHandle> keeps = pass_through_attributes;
        keeps.insert(keeps.end(), original_attributes.begin(), original_attributes.end());
        mesh.clear_attributes(keeps);
    }

    cache.write_mesh(mesh, options.output);
}

} // namespace wmtk::components
