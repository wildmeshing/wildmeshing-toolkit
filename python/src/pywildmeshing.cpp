#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <nlohmann/json.hpp>
#include <pybind11_json/pybind11_json.hpp>
#include <wmtk/utils/Logger.hpp>

#include <deque>

#include "mesh_conversion.hpp"

// components
#include <component_specs.hpp>
#include <components_include.hpp>
#include <components_spec_include.hpp>

namespace py = pybind11;
using namespace pybind11::literals;

namespace wmtk::python {
// The components' own bindings, those that have them (add_component):
// python/src/<name>_bindings.cpp defines bind_<name>.
#define WMTK_PYTHON_BINDINGS(name) void bind_##name(pybind11::module_& module);
#include <components_python.hpp>
#undef WMTK_PYTHON_BINDINGS
} // namespace wmtk::python

namespace {

const wmtk::component_specs::SpecMap& spec_map()
{
    static const wmtk::component_specs::SpecMap components_spec_map = [] {
        wmtk::component_specs::SpecMap components_spec_map;
#include <components_spec_map.hpp>
        return components_spec_map;
    }();
    return components_spec_map;
}

// As wmtk_app does: most components ignore an option their spec does not have.
void warn_unknown_options(const std::string& name, const nlohmann::json& options)
{
    const auto it = spec_map().find(name);
    if (it == spec_map().end()) return;
    for (const auto& key : wmtk::component_specs::unknown_options(it->second(), options)) {
        wmtk::logger().warn("{} has no option `{}` (see help(wildmeshing.{}))", name, key, name);
    }
}

} // namespace


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

    warn_unknown_options(app_str, j);

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
    // The docstrings, built from the specs, have to outlive the module definition.
    static std::deque<std::string> docstrings;
    for (const auto& [name, component] : memory_components) {
        const auto spec = spec_map().find(name);
        docstrings.push_back(
            (spec == spec_map().end()
                 ? std::string()
                 : wmtk::component_specs::describe(name, spec->second()) + "\n") +
            "With meshes: run on named NumPy mesh dictionaries and return (meshes, report) "
            "without file I/O. Component calls run one at a time per process.");
        m.def(
            name.c_str(),
            [name, component](const py::dict& options, const py::dict& meshes) {
                const nlohmann::json params = wmtk::python::options(options);
                warn_unknown_options(name, params);
                return wmtk::python::call_component(component, params, meshes);
            },
            "options"_a,
            "meshes"_a,
            docstrings.back().c_str());
        m.def(
            name.c_str(),
            [name, component = components_map.at(name)](const py::dict& options) {
                nlohmann::json params = wmtk::python::options(options);
                if (!params.contains("application")) params["application"] = name;
                warn_unknown_options(name, params);
                py::gil_scoped_release release;
                component(std::move(params));
            },
            "options"_a,
            "Without meshes: run on files, as wmtk_app does with the same options.");
    }

    m.def(
        "components",
        [] {
            std::vector<std::string> names;
            for (const auto& [name, spec] : spec_map()) names.push_back(name);
            return names;
        },
        "The names of the components, each a function of this module.");
    m.def(
        "spec",
        [](const std::string& name) -> py::object {
            const auto it = spec_map().find(name);
            if (it == spec_map().end()) throw py::value_error("Unknown component " + name);
            return it->second();
        },
        "name"_a,
        "A component's JSON spec: its option rules, as the reference documentation lists them.");
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
            warn_unknown_options(name, params);
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
