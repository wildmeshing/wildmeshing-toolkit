
#include <wmtk/Mesh.hpp>
#include <nlohmann/json.hpp>

#include <wmtk/components/multimesh/from_boundary.hpp>
#include <wmtk/components/multimesh/from_facet_bijection.hpp>
#include <wmtk/components/multimesh/axis_aligned_fusion.hpp>

#include <wmtk/components/input/utils/get_attribute.hpp>
#include <wmtk/components/input/input.hpp>
#include <wmtk/components/input/InputOptions.hpp>

namespace {

void make_boundary(wmtk::Mesh& m, const nlohmann::json& bdry_cfg) {

        std::vector<wmtk::attribute::MeshAttributeHandle> transferred_attrs;
        if(bdry_cfg.contains("transferred_attributes")) {
           for(const nlohmann::json& attr:  bdry_cfg["transferred_attributes"]) {
               transferred_attrs.emplace_back(wmtk::components::input::utils::get_attribute(m, attr));
           }
        }
        int dim = bdry_cfg.contains("simplex") ? bdry_cfg["simplex"].get<int>() : m.top_cell_dimension() - 1;

        wmtk::PrimitiveType pt = wmtk::get_primitive_type_from_id(dim);

        const std::string name = bdry_cfg.contains("attribute_name") ? bdry_cfg["attribute_name"].get<std::string>() : std::string("is_boundary");
        const int value = bdry_cfg.contains("attribute_value") ? bdry_cfg["attribute_value"].get<int>() : 1;

        wmtk::components::multimesh::from_boundary(m, pt, name, value, transferred_attrs);


    }
}
auto make_fusion(wmtk::Mesh& m, const nlohmann::json& js) {

    std::vector<bool> cfg = js;

    return wmtk::components::multimesh::axis_aligned_fusion(m,js);

}


void make_facet_bijection(wmtk::Mesh& m, const nlohmann::json& js) {
    auto opts = js.get<wmtk::components::input::InputOptions>();
    auto child = wmtk::components::input::input(opts).root().shared_from_this();

    wmtk::components::multimesh::from_facet_bijection(m,*child);
}

std::shared_ptr<wmtk::Mesh> make_multimesh(wmtk::Mesh& m, const nlohmann::json& multimesh_config) {

    if(multimesh_config.contains("boundary")) {
        make_boundary(m,multimesh_config["boundary"]);
        return m.shared_from_this();
    } else 
    if(multimesh_config.contains("fusion")) {
        return make_fusion(m, multimesh_config["fusion"]);
    } else if(multimesh_config.contains("facet_bijection")) {
        make_facet_bijection(m,multimesh_config["facet_bijection"]);
        return m.shared_from_this();
        

    }
    return m.shared_from_this();
}
