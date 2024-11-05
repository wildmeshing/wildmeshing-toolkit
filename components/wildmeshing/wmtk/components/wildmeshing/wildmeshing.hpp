#pragma once

#include <string>
#include <wmtk/Mesh.hpp>

#include "internal/WildmeshingOptions.hpp"


namespace wmtk::components {

std::vector<std::pair<std::shared_ptr<Mesh>, std::string>> wildmeshing(
    const WildMeshingOptions& option);

} // namespace wmtk::components
