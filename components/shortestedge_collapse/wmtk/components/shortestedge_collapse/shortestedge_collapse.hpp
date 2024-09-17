#pragma once
#include <wmtk/Mesh.hpp>

namespace wmtk::components {

std::shared_ptr<Mesh> shortestedge_collapse(
    const std::shared_ptr<Mesh>& position_mesh,
    const std::string& position_handle_name,
    const std::shared_ptr<Mesh>& inversion_mesh,
    const std::string& inversion_position_handle_name,
    bool update_other_position,
    const double length_rel,
    bool lock_boundary,
    double envelope_size);

} // namespace wmtk::components
