#pragma once

#include <wmtk/Mesh.hpp>


namespace wmtk::components::internal {

void write(
    const std::shared_ptr<Mesh>& mesh,
    const std::string& out_dir,
    const std::string& name,
    const std::string& vname,
    const int64_t index,
    const bool intermediate_output);

} // namespace wmtk::components::internal