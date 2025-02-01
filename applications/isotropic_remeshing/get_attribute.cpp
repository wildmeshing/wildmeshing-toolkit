
#include <wmtk/Mesh.hpp>
#include <nlohmann/json.hpp>

#include <wmtk/components/multimesh/from_boundary.hpp>
#include <wmtk/components/multimesh/axis_aligned_fusion.hpp>


wmtk::attribute::MeshAttributeHandle get_attribute(wmtk::Mesh& m, const nlohmann::json& js) {

    const std::string mesh = js.contains("mesh")
}
wmtk::attribute::MeshAttributeHandle get_attribute(wmtk::components::input::MeshCollection& nmm, const nlohmann::json& js) {
}
