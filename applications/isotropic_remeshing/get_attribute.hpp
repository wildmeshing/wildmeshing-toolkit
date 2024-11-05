#pragma once
#include <nlohmann/json_fwd.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
namespace wmtk {
    class Mesh;
}



wmtk::attribute::MeshAttributeHandle get_attribute(wmtk::Mesh& m, const nlohmann::json& js);
wmtk::attribute::MeshAttributeHandle get_attribute(wmtk::components::input::MeshCollection& nmm, const nlohmann::json& js);

