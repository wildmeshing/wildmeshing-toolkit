#include <pybind11/pybind11.h>

#include <pybind11/eigen.h>
#include <pybind11/iostream.h>
#include <pybind11/stl.h>
#include <pybind11_json/pybind11_json.hpp>

#include <nlohmann/json.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/simwild/expression_parser/Parser.hpp>
#include "mesh_conversion.hpp"

// components
#include "../components_include.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

namespace wmtk::python {
// The components' own bindings, those that have them (add_component):
// app/pywildmeshing/<name>_bindings.cpp defines bind_<name>.
#define WMTK_PYTHON_BINDINGS(name) void bind_##name(pybind11::module_& module);
#include "../components_python.hpp"
#undef WMTK_PYTHON_BINDINGS
} // namespace wmtk::python


void wmtk_wrapper(const py::dict& obj)
{
    const nlohmann::json j = wmtk::python::options(obj);

    using namespace wmtk;
    std::map<std::string, std::function<void(nlohmann::json)>> components_map;
    // include auto-generated map
#include "../components_map.hpp"

    // make sure input file contains the application name
    if (!j.contains("application")) {
        logger().info("App not found");
        std::cerr << "Application not found" << std::endl;
        log_and_throw_error("JSON input file must contain entry `application`.");
    }

    std::string app_str = j["application"];
    if (components_map.count(app_str) == 0) {
        log_and_throw_error("Application {} unknown", app_str);
    }

    // execute
    components_map[app_str](j);
}

// Parsed tag expression evaluable from Python over sets of group NAMES.
// Single source of truth for the grammar shared with the Python-side ops
// (simwild.polyfem_ops.mesh_core delegates here instead of mirroring the
// parser).
class PyExpression
{
public:
    PyExpression(const std::string& expr, const std::vector<std::string>& known_names)
    {
        int64_t i = 0;
        for (const auto& n : known_names) {
            m_name_to_id.emplace(n, i++);
        }
        m_expr = wmtk::components::simwild::expression_parser::parse(expr, m_name_to_id);
    }

    bool eval(const std::set<std::string>& tags) const
    {
        // Names outside known_names still count toward the tag set (they can
        // never satisfy a name atom, but `_` = emptiness must see them) —
        // mirrors SimWildMesh::string_set_to_cell_tag assigning fresh ids.
        wmtk::components::simwild::CellTag ct;
        int64_t fresh = static_cast<int64_t>(m_name_to_id.size());
        for (const auto& t : tags) {
            const auto it = m_name_to_id.find(t);
            ct.insert(it != m_name_to_id.end() ? it->second : fresh++);
        }
        return m_expr->eval(ct);
    }

    bool has_unknown_names() const
    {
        // unknown identifiers parse to TagExpr(-1)
        return m_expr->tags_involved().count(-1) > 0;
    }

private:
    std::map<std::string, int64_t> m_name_to_id;
    wmtk::components::simwild::expression_parser::ExpressionPtr m_expr;
};

PYBIND11_MODULE(_core, m, py::mod_gil_not_used())
{
    m.doc() = "Python bindings for the Wildmeshing-Toolkit"; // optional module docstring
    m.def("wildmeshing", &wmtk_wrapper, "options"_a, "Wildmeshing-Toolkit application");

    std::map<std::string, std::function<void(nlohmann::json)>> components_map;
#include "../components_map.hpp"
    std::map<std::string, wmtk::python::MemoryComponent> memory_components;
#include "../components_memory_map.hpp"
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
#include "../components_python.hpp"
#undef WMTK_PYTHON_BINDINGS

    py::class_<PyExpression>(m, "Expression")
        .def(
            py::init<const std::string&, const std::vector<std::string>&>(),
            "expr"_a,
            "known_names"_a,
            "Parse a tag expression (names, &, |, !, parentheses, _) "
            "against the given known names.")
        .def("eval", &PyExpression::eval, "tags"_a, "Evaluate against a set of tag names.")
        .def(
            "has_unknown_names",
            &PyExpression::has_unknown_names,
            "True if the expression references names outside known_names.");
}
