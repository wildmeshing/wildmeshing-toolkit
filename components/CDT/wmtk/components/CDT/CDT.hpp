#pragma once
#include <wmtk/Mesh.hpp>

namespace wmtk::components {

std::shared_ptr<Mesh>
CDT(const std::shared_ptr<Mesh>& input, const bool inner_only, const bool rational_output);

} // namespace wmtk::components
