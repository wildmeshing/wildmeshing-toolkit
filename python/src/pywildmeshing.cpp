#include <pybind11/pybind11.h>

#include <nlohmann/json.hpp>
#include <wmtk/utils/Logger.hpp>

#include "mesh_conversion.hpp"

// components
#include <components_include.hpp>

namespace py = pybind11;
using namespace pybind11::literals;

namespace wmtk::python {
// The components' own bindings, those that have them (add_component):
// python/src/<name>_bindings.cpp defines bind_<name>.
#define WMTK_PYTHON_BINDINGS(name) void bind_##name(pybind11::module_& module);
#include <components_python.hpp>
#undef WMTK_PYTHON_BINDINGS
} // namespace wmtk::python


void wmtk_wrapper(const py::dict& obj)
{
    const nlohmann::json j = wmtk::python::options(obj);

    using namespace wmtk;
    std::map<std::string, std::function<void(nlohmann::json)>> components_map;
    // include auto-generated map
#include <components_map.hpp>

    // make sure input file contains the application name
    if (!j.contains("application")) {
        log_and_throw_error("JSON input file must contain entry `application`.");
    }

    std::string app_str = j["application"];
    if (components_map.count(app_str) == 0) {
        log_and_throw_error("Application {} unknown", app_str);
    }

    // execute
    components_map[app_str](j);
}

PYBIND11_MODULE(_core, m, py::mod_gil_not_used())
{
    m.doc() = "Python bindings for the Wildmeshing-Toolkit"; // optional module docstring
    m.def("wildmeshing", &wmtk_wrapper, "options"_a, "Wildmeshing-Toolkit application");

    std::map<std::string, std::function<void(nlohmann::json)>> components_map;
#include <components_map.hpp>
    std::map<std::string, wmtk::python::MemoryComponent> memory_components;
#include <components_memory_map.hpp>
    for (const auto& [name, component] : memory_components) {
        m.def(
            name.c_str(),
            [component](const py::dict& options, const py::dict& meshes) {
                return wmtk::python::call_component(
                    component,
                    wmtk::python::options(options),
                    meshes);
            },
            "options"_a,
            "meshes"_a,
            "Run on named NumPy mesh dictionaries and return (meshes, report) without file I/O. "
            "Component calls run one at a time per process.");
        m.def(
            name.c_str(),
            [name, component = components_map.at(name)](const py::dict& options) {
                nlohmann::json params = wmtk::python::options(options);
                if (!params.contains("application")) params["application"] = name;
                py::gil_scoped_release release;
                component(std::move(params));
            },
            "options"_a,
            "Run the existing file-based component interface.");
    }
    m.def(
        "wildmeshing",
        [memory_components](const py::dict& options, const py::dict& meshes) {
            const nlohmann::json params = wmtk::python::options(options);
            if (!params.contains("application"))
                throw std::runtime_error("JSON input must contain entry `application`.");
            const auto name = params.at("application").get<std::string>();
            const auto it = memory_components.find(name);
            if (it == memory_components.end())
                throw std::runtime_error("Application " + name + " unknown");
            return wmtk::python::call_component(it->second, params, meshes);
        },
        "options"_a,
        "meshes"_a,
        "Dispatch an in-memory component using options['application'].");
    wmtk::python::bind_mesh_io(m);
#define WMTK_PYTHON_BINDINGS(name) wmtk::python::bind_##name(m);
#include <components_python.hpp>
#undef WMTK_PYTHON_BINDINGS
}
