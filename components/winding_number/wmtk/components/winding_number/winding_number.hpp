#pragma once

#include <wmtk/Mesh.hpp>

namespace wmtk::components {

std::shared_ptr<Mesh> winding_number(
    const std::shared_ptr<Mesh>& query_mesh_ptr,
    const std::shared_ptr<Mesh>& surface_ptr);

}