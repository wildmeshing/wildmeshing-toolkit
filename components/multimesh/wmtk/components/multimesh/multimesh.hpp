#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components {

std::pair<std::shared_ptr<Mesh>, std::shared_ptr<Mesh>> multimesh(
    const std::string type,
    const std::shared_ptr<Mesh>& mesh,
    const std::shared_ptr<Mesh>& child,
    const std::string position_handle_name,
    const std::string tag_name,
    const int64_t tag_value,
    const int64_t primitive);

} // namespace wmtk::components
