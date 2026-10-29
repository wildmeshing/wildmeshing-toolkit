#pragma once

#include <pybind11/pybind11.h>
#include <wmtk/MeshData.hpp>

namespace wmtk::python {
using MemoryComponent = ComponentResult (*)(nlohmann::json, const MeshMap&);

// pybind11_json turns bytes into base64 text and has no binary values: options() makes the bytes
// in a call's options (a history) JSON binary values, and report() those of a report
// (the history a memory call returns) bytes again. Everything else converts as pybind11_json does.
nlohmann::json options(const pybind11::handle& value);
pybind11::object report(const nlohmann::json& j);
pybind11::tuple
call_component(MemoryComponent component, nlohmann::json options, const pybind11::dict& meshes);
void bind_mesh_io(pybind11::module_& module);
} // namespace wmtk::python
