#pragma once

#include <pybind11/pybind11.h>
#include <wmtk/MeshData.hpp>

namespace wmtk::python {
using MemoryComponent = ComponentResult (*)(nlohmann::json, const MeshMap&);

pybind11::tuple
call_component(MemoryComponent component, nlohmann::json options, const pybind11::dict& meshes);
void bind_mesh_io(pybind11::module_& module);
} // namespace wmtk::python
