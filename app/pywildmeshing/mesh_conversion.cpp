#include "mesh_conversion.hpp"

#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>
#include <pybind11_json/pybind11_json.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/io/mesh_io.hpp>

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
    check_keys(input, {"V", "cells", "vertex_attributes"});
    MeshData result;
    const auto vertices = array(input["V"]);
    if (vertices.dtype().kind() == 'f' && vertices.dtype().itemsize() <= 8)
        result.V = matrix<double>(vertices);
    else {
        MeshAttribute coordinates;
        coordinates.values = integers(vertices);
        result.V = coordinates.doubles();
    }
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
    return py::dict(
        "V"_a = numpy(input.V),
        "vertex_attributes"_a = attributes(input.vertex_attributes),
        "cells"_a = cells);
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
          "input_edges",
          "input_points",
          "surface_mesh",
          "uv_mesh",
          "envelope",
          "tet_mesh",
          "features"}) {
        const auto it = options.find(role);
        if (it == options.end() || it->is_null()) continue;
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
void bind_mesh_io(py::module_& module)
{
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
        "volume_history_output) as the bytes a memory call returns in its report's 'history' "
        "(or 'volume_history') and takes as 'history'.");
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
        "Save the bytes of a history (a memory call's report['history'], or "
        "report['volume_history']) as the file the file route writes.");
}
} // namespace wmtk::python
