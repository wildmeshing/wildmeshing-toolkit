#pragma once
#include <nlohmann/json.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>

namespace wmtk {
class Mesh;
namespace components::input {
class MeshCollection;
}
} // namespace wmtk

namespace wmtk::components::input::utils {
    //
    // assuems json has 
    // * name (str for attribute name)
    // * mesh (mesh path, mesh only only overload does not use this)
    // * type (str double/int/char/rational
    // * simplex (int, dimension attribute belongs to

wmtk::attribute::MeshAttributeHandle get_attribute(const Mesh& m, const nlohmann::json& js);
wmtk::attribute::MeshAttributeHandle get_attribute(
    const wmtk::components::input::MeshCollection& m,
    const nlohmann::json& js);
} // namespace wmtk::components::input::utils
