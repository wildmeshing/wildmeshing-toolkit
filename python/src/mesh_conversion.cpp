#include "mesh_conversion.hpp"

#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11/warnings.h>
#include <wmtk/RationalPositions.h>
#include <pybind11_json/pybind11_json.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/io/mesh_io.hpp>

#include <algorithm>
#include <cstring>
#include <limits>
#include <set>
#include <stdexcept>

namespace py = pybind11;
using namespace pybind11::literals;

namespace wmtk::python {
namespace {
const std::map<std::string, CellType> cell_types = {
    {"point", CellType::Point},
    {"edge", CellType::Edge},
    {"triangle", CellType::Triangle},
    {"quad", CellType::Quad},
    {"tetrahedron", CellType::Tetrahedron},
    {"prism", CellType::Prism},
    {"pyramid", CellType::Pyramid}};

void check_keys(const py::dict& value, const std::set<std::string>& allowed)
{
    for (auto item : value) {
        if (!py::isinstance<py::str>(item.first)) throw py::type_error("Keys must be strings");
        const auto key = py::cast<std::string>(item.first);
        if (!allowed.count(key)) throw py::value_error("Unknown mesh data field: " + key);
    }
}
} // namespace

py::dict dictionary(py::handle value, const std::string& name)
{
    if (!py::isinstance<py::dict>(value)) throw py::type_error(name + " must be a dictionary");
    return py::reinterpret_borrow<py::dict>(value);
}
py::array array(py::handle value)
{
    if (!py::isinstance<py::array>(value)) throw py::type_error("Mesh arrays must be NumPy arrays");
    return py::reinterpret_borrow<py::array>(value);
}
DataMatrix<int64_t> integers(const py::array& value, const bool allow_vector)
{
    const auto dtype = value.dtype();
    if ((dtype.kind() != 'i' && dtype.kind() != 'u') || dtype.itemsize() > 8)
        throw py::type_error("Connectivity must have an integer dtype");
    if (dtype.kind() == 'u' && dtype.itemsize() == 8) {
        const auto values = matrix<uint64_t>(value, allow_vector);
        for (Eigen::Index i = 0; i < values.size(); ++i)
            if (values.data()[i] > uint64_t(std::numeric_limits<int64_t>::max()))
                throw py::value_error("Integer mesh array exceeds Int64 range");
    }
    return matrix<int64_t>(value, allow_vector);
}

namespace {
// mesh["V_rational"]: rows, an integer vector, and values, one rational per coordinate of each
// row -- fractions.Fraction, or anything Fraction() takes exactly (an integer, a float, a "p/q"
// string) -- as MeshData::V_exact holds them, GMP's base-10 "p/q".
ExactVertices exact_vertices(const py::dict& input, const Eigen::Index cols)
{
    check_keys(input, {"rows", "values"});
    if (!input.contains("rows") || !input.contains("values"))
        throw py::value_error("V_rational needs rows and values");
    ExactVertices result;
    const auto rows = integers(array(input["rows"]), true);
    if (rows.cols() != 1) throw py::value_error("V_rational rows must be a vector");
    result.rows.assign(rows.data(), rows.data() + rows.size());
    const auto values =
        py::module_::import("numpy").attr("asarray")(input["values"], "dtype"_a = "object");
    const auto shape = values.attr("shape").cast<std::vector<Eigen::Index>>();
    if (shape.size() != 2 || shape[0] != Eigen::Index(result.rows.size()) || shape[1] != cols)
        throw py::value_error("V_rational values must have one row per row and V's columns");
    const auto fraction = py::module_::import("fractions").attr("Fraction");
    for (const auto row : values.attr("tolist")())
        for (const auto value : row) result.values.push_back(py::str(fraction(value)));
    return result;
}
py::dict exact_vertices(const ExactVertices& input, const Eigen::Index cols)
{
    const auto fraction = py::module_::import("fractions").attr("Fraction");
    const size_t n = input.rows.size();
    py::array_t<int64_t> rows(static_cast<py::ssize_t>(n));
    std::copy(input.rows.begin(), input.rows.end(), rows.mutable_data());
    py::object values =
        py::module_::import("numpy").attr("empty")(py::make_tuple(n, cols), "dtype"_a = "object");
    for (size_t i = 0; i < n; ++i)
        for (Eigen::Index k = 0; k < cols; ++k)
            values[py::make_tuple(i, k)] = fraction(input.values[i * cols + k]);
    return py::dict("rows"_a = rows, "values"_a = values);
}
MeshAttributes attributes(const py::dict& input)
{
    MeshAttributes result;
    for (auto item : input) {
        if (!py::isinstance<py::str>(item.first))
            throw py::type_error("Attribute names must be strings");
        MeshAttribute attribute;
        py::handle value = item.second;
        if (py::isinstance<py::dict>(value)) {
            const auto field = dictionary(value, "Attribute");
            check_keys(field, {"values", "labels"});
            value = field["values"];
            if (field.contains("labels"))
                attribute.labels = field["labels"].cast<std::vector<std::string>>();
        }
        const auto values = array(value);
        const auto dtype = values.dtype();
        if (dtype.kind() == 'f' && dtype.itemsize() == 8)
            attribute.values = matrix<double>(values, true);
        else if (dtype.kind() == 'i' && dtype.itemsize() == 8)
            attribute.values = matrix<int64_t>(values, true);
        else if (dtype.kind() == 'u' && dtype.itemsize() == 1)
            attribute.values = matrix<uint8_t>(values, true);
        else
            throw py::type_error("Attribute dtype must be float64, int64, or uint8");
        result.emplace(py::cast<std::string>(item.first), std::move(attribute));
    }
    return result;
}
} // namespace

MeshData mesh_data(const py::dict& input)
{
    check_keys(input, {"V", "V_rational", "cells", "vertex_attributes"});
    MeshData result;
    const auto vertices = array(input["V"]);
    if (vertices.dtype().kind() == 'f' && vertices.dtype().itemsize() <= 8)
        result.V = matrix<double>(vertices);
    else {
        MeshAttribute coordinates;
        coordinates.values = integers(vertices);
        result.V = coordinates.doubles();
    }
    if (input.contains("V_rational"))
        result.V_exact =
            exact_vertices(dictionary(input["V_rational"], "V_rational"), result.V.cols());
    if (input.contains("vertex_attributes"))
        result.vertex_attributes =
            attributes(dictionary(input["vertex_attributes"], "vertex_attributes"));
    const auto cells = dictionary(input["cells"], "cells");
    for (auto item : cells) {
        if (!py::isinstance<py::str>(item.first))
            throw py::type_error("Cell types must be strings");
        const auto name = py::cast<std::string>(item.first);
        const auto type = cell_types.find(name);
        if (type == cell_types.end()) throw py::value_error("Unknown cell type: " + name);
        const auto block = dictionary(item.second, "Cell block");
        check_keys(block, {"connectivity", "attributes"});
        auto& data = result.cells[type->second];
        data.connectivity = integers(array(block["connectivity"]));
        if (block.contains("attributes"))
            data.attributes = attributes(dictionary(block["attributes"], "attributes"));
    }
    return result;
}
namespace {
py::dict attributes(const MeshAttributes& input)
{
    py::dict result;
    for (const auto& [name, attribute] : input)
        result[py::str(name)] = py::dict(
            "values"_a =
                std::visit([](const auto& values) { return numpy(values); }, attribute.values),
            "labels"_a = attribute.labels);
    return result;
}
} // namespace

py::dict mesh_data(const MeshData& input)
{
    py::dict cells;
    for (const auto& [name, type] : cell_types) {
        const auto it = input.cells.find(type);
        if (it != input.cells.end())
            cells[py::str(name)] = py::dict(
                "connectivity"_a = numpy(it->second.connectivity),
                "attributes"_a = attributes(it->second.attributes));
    }
    py::dict result(
        "V"_a = numpy(input.V),
        "vertex_attributes"_a = attributes(input.vertex_attributes),
        "cells"_a = cells);
    // Only with vertices that could not be rounded: every other mesh is exactly its V.
    if (!input.V_exact.empty())
        result["V_rational"] = exact_vertices(input.V_exact, input.V.cols());
    return result;
}

nlohmann::json options(const py::handle& value)
{
    if (py::isinstance<py::bytes>(value)) {
        const auto data = py::cast<std::string>(value);
        return nlohmann::json::binary(std::vector<uint8_t>(data.begin(), data.end()));
    }
    if (py::isinstance<py::dict>(value)) {
        nlohmann::json j = nlohmann::json::object();
        for (auto item : py::reinterpret_borrow<py::dict>(value)) {
            if (!py::isinstance<py::str>(item.first)) throw py::type_error("Keys must be strings");
            j[py::cast<std::string>(item.first)] = options(item.second);
        }
        return j;
    }
    if (py::isinstance<py::list>(value) || py::isinstance<py::tuple>(value)) {
        nlohmann::json j = nlohmann::json::array();
        for (auto item : value) j.push_back(options(item));
        return j;
    }
    return pyjson::to_json(value);
}
py::object report(const nlohmann::json& j)
{
    if (j.is_binary()) {
        const auto& data = j.get_binary();
        return py::bytes(reinterpret_cast<const char*>(data.data()), data.size());
    }
    if (j.is_object()) {
        py::dict d;
        for (auto it = j.begin(); it != j.end(); ++it) d[py::str(it.key())] = report(it.value());
        return d;
    }
    if (j.is_array()) {
        py::list l;
        for (const auto& v : j) l.append(report(v));
        return l;
    }
    return pyjson::from_json(j);
}
py::tuple call_component(MemoryComponent component, nlohmann::json options, const py::dict& meshes)
{
    MeshMap inputs;
    // Convert only named selections; unrelated dictionary entries stay untouched.
    auto select = [&](const nlohmann::json& name) {
        if (!name.is_string()) throw py::type_error("Mesh selectors must be strings");
        const auto key = name.get<std::string>();
        if (inputs.count(key)) return;
        if (!meshes.contains(py::str(key))) throw std::runtime_error("Unknown input mesh: " + key);
        inputs.emplace(key, mesh_data(dictionary(meshes[py::str(key)], "MeshData")));
    };
    for (const auto* role :
         {"input",
          "surface_mesh",
          "uv_mesh",
          "envelope",
          "tet_mesh",
          "output_tet_mesh",
          "features",
          "sizing_mesh"}) {
        const auto it = options.find(role);
        if (it == options.end() || it->is_null()) continue;
        if (it->is_string() && it->get<std::string>().empty()) continue; // "": none
        if (it->is_array())
            for (const auto& name : *it) select(name);
        else
            select(*it);
    }
    ComponentResult result;
    {
        py::gil_scoped_release release;
        result = component(std::move(options), inputs);
    }
    py::dict outputs;
    for (const auto& [name, data] : result.first) outputs[py::str(name)] = mesh_data(data);
    return py::make_tuple(outputs, report(result.second));
}
namespace {
// UnroundedVerticesWarning, made by bind_mesh_io(); the module holds it for good.
PyObject* unrounded_warning = PyExc_RuntimeWarning;
void warn_unrounded(const std::string& message)
{
    // Stack level 1: the line of the call into the module.
    py::warnings::warn(message.c_str(), unrounded_warning, 1);
}
} // namespace

void clear_unrounded_output()
{
    take_unrounded_output();
}
void warn_unrounded_output(const bool memory)
{
    const auto found = take_unrounded_output();
    if (found.unrounded == 0) return;
    warn_unrounded(
        found.component + ": " + std::to_string(found.unrounded) +
        " vertices could not be rounded. " +
        (memory ? "Their exact positions are in the output's V_rational and its V holds their "
                  "rounding only, in which cells can be degenerate or inverted: remesh from "
                  "memory, passing the mesh with its V_rational, before using V or writing a file."
                : "The written mesh holds their rounding only, in which cells can be degenerate "
                  "or inverted: run it in memory and remesh the result to keep their exact "
                  "positions."));
}

void bind_mesh_io(py::module_& module)
{
    unrounded_warning =
        py::warnings::new_warning_type(module, "UnroundedVerticesWarning").release().ptr();
    module.attr("UnroundedVerticesWarning").attr("__doc__") =
        "A result has vertices that could not be rounded: in memory their exact positions are "
        "in the mesh's V_rational, and its V (and any file written from it) holds their rounding "
        "only, in which cells can be degenerate or inverted.";
    module.def(
        "read_mesh",
        [](const std::filesystem::path& path) {
            MeshData result;
            {
                py::gil_scoped_release release;
                result = io::read_mesh(path);
            }
            return mesh_data(result);
        },
        "path"_a,
        "Read an existing supported mesh format into dictionaries of owning NumPy arrays.");
    module.def(
        "write_mesh",
        [](const std::filesystem::path& path, const py::dict& input) {
            const auto data = mesh_data(input);
            if (!data.V_exact.empty())
                warn_unrounded(
                    "write_mesh: " + std::to_string(data.V_exact.rows.size()) +
                    " vertices' exact positions (V_rational) are not written: the file holds "
                    "their rounding only, in which cells can be degenerate or inverted.");
            py::gil_scoped_release release;
            io::write_mesh(path, data);
        },
        "path"_a,
        "mesh"_a,
        "Save MeshData through the shared MSH or existing VTU writer.");
    module.def(
        "read_history",
        [](const std::filesystem::path& path) {
            std::vector<uint8_t> bytes;
            {
                py::gil_scoped_release release;
                bytes = history::read_history(path);
            }
            return py::bytes(reinterpret_cast<const char*>(bytes.data()), bytes.size());
        },
        "path"_a,
        "Read a history file (a component's history_output, of a surface or of curves, or its "
        "volume_history_output or curve_history_output) as the bytes a memory call returns in "
        "its report's 'history' ('volume_history', 'curve_history') and takes as 'history' "
        "('volume_history').");
    module.def(
        "write_history",
        [](const std::filesystem::path& path, const py::bytes& history) {
            const auto data = py::cast<std::string>(history);
            const std::vector<uint8_t> bytes(data.begin(), data.end());
            py::gil_scoped_release release;
            history::write_history(path, bytes);
        },
        "path"_a,
        "history"_a,
        "Save the bytes of a history (a memory call's report['history'], "
        "report['volume_history'] or report['curve_history']) as the file the file route "
        "writes.");
}
} // namespace wmtk::python
