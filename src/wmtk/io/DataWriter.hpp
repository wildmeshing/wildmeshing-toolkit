#pragma once
#include <paraviewo/VTUWriter.hpp>
#include <set>
#include <stdexcept>
#include <wmtk/MeshData.hpp>

namespace wmtk::io {
// An output observer shared by the legacy exporters and their in-memory adapters.
// File calls delegate to the original writer, retaining its formatting and filenames.
class DataWriter
{
    MeshMap* output;
    paraviewo::VTUWriter file;
    MeshAttributes points, fields;

public:
    explicit DataWriter(MeshMap* meshes = nullptr)
        : output(meshes)
    {}
    void add_field(const std::string& name, const Eigen::MatrixXd& data)
    {
        if (output)
            points[name].values = DataMatrix<double>(data);
        else
            file.add_field(name, data);
    }
    void add_integer_field(const std::string& name, const DataMatrix<int64_t>& values)
    {
        if (output)
            points[name].values = values;
        else {
            MeshAttribute field;
            field.values = values;
            file.add_field(name, field.doubles());
        }
    }
    void add_cell_field(const std::string& name, const Eigen::MatrixXd& data)
    {
        if (output)
            fields[name].values = DataMatrix<double>(data);
        else
            file.add_cell_field(name, data);
    }
    bool write_mesh(
        const std::string& path,
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& C,
        paraviewo::CellType type)
    {
        if (!output) return file.write_mesh(path, V, C, type);
        std::vector<paraviewo::CellElement> cells(C.rows());
        for (Eigen::Index i = 0; i < C.rows(); ++i) {
            cells[i].ctype = type;
            for (Eigen::Index j = 0; j < C.cols(); ++j) cells[i].vertices.push_back(C(i, j));
        }
        return write_mesh(path, V, cells);
    }
    bool write_mesh(
        const std::string& path,
        const Eigen::MatrixXd& V,
        const std::vector<paraviewo::CellElement>& cells)
    {
        if (!output) return file.write_mesh(path, V, cells);
        MeshData data;
        data.V = DataMatrix<double>::Zero(V.rows(), 3);
        data.V.leftCols(V.cols()) = V;
        data.vertex_attributes = points;
        std::map<CellType, std::vector<size_t>> rows;
        for (size_t i = 0; i < cells.size(); ++i) {
            const auto& c = cells[i];
            const auto type = static_cast<CellType>(
                paraviewo::paraview_tags::VTKTag(int(c.vertices.size()), c.ctype));
            auto& block = data.cells[type].connectivity;
            block.conservativeResize(block.rows() + 1, cell_size(type));
            for (size_t j = 0; j < c.vertices.size(); ++j)
                block(block.rows() - 1, j) = c.vertices[j];
            rows[type].push_back(i);
        }
        for (const auto& [type, indices] : rows)
            for (const auto& [name, a] : fields) {
                auto values = a.doubles();
                DataMatrix<double> sub(indices.size(), values.cols());
                for (size_t r = 0; r < indices.size(); ++r) sub.row(r) = values.row(indices[r]);
                data.cells[type].attributes[name].values = std::move(sub);
            }
        data.validate();
        output->insert_or_assign(path, std::move(data));
        // Like VTUWriter::write_mesh, which clears its fields after every write.
        points.clear();
        fields.clear();
        return true;
    }
};
// Export arrays are sometimes indexed by reserved slots. Remove dead slots without
// consolidating or otherwise mutating the algorithm's mesh, and keep every field aligned.
inline void compact_vertices(MeshData& data, const std::vector<size_t>& rows)
{
    DataMatrix<double> V(rows.size(), 3);
    std::vector<int64_t> remap(data.V.rows(), -1);
    for (size_t i = 0; i < rows.size(); ++i) {
        V.row(i) = data.V.row(rows[i]);
        remap[rows[i]] = i;
    }
    data.V = std::move(V);
    for (auto& [name, a] : data.vertex_attributes)
        a.values = std::visit(
            [&](const auto& values) -> AttributeValues {
                using M = std::decay_t<decltype(values)>;
                M compact(rows.size(), values.cols());
                for (size_t i = 0; i < rows.size(); ++i) compact.row(i) = values.row(rows[i]);
                return compact;
            },
            a.values);
    for (auto& [type, block] : data.cells)
        for (Eigen::Index i = 0; i < block.connectivity.size(); ++i) {
            const auto v = remap.at(block.connectivity.data()[i]);
            if (v < 0) throw std::runtime_error("Export references an inactive vertex");
            block.connectivity.data()[i] = v;
        }
    data.validate();
}
template <typename Mesh>
void compact_slot_outputs(const Mesh& mesh, MeshMap& outputs)
{
    std::vector<size_t> rows;
    for (const auto& v : mesh.get_vertices()) rows.push_back(v.vid(mesh));
    for (auto& [name, data] : outputs) compact_vertices(data, rows);
}
// The result key of each memory export. The exporters keep their legacy file naming,
// <stem><suffix>.vtu, and the memory calls use the stem "mesh". An empty key drops an export
// that the caller returns in another form. simwild's <output>_history_input.msh, the mesh its
// surface history starts from, is the one .msh among them.
using OutputKeys = std::map<std::string, std::string>;
inline OutputKeys default_output_keys()
{
    return {
        {"mesh.vtu", "mesh"},
        {"mesh_surf.vtu", "surface"},
        {"mesh_edge.vtu", "edges"},
        {"mesh_off.vtu", "offset"},
        {"mesh_front.vtu", "offset"},
        {"mesh_offset_faces.vtu", "offset"},
        {"mesh_history_input.msh", "history_input"}};
}
// Rename the exports into public result keys. An export missing from `keys`, or two exports
// with the same key, is an error rather than a silently dropped output.
inline MeshMap named_outputs(MeshMap raw, const OutputKeys& keys = default_output_keys())
{
    MeshMap result;
    for (auto& [path, data] : raw) {
        const auto key = keys.find(path);
        if (key == keys.end()) throw std::runtime_error("Unexpected memory export: " + path);
        if (key->second.empty()) continue;
        if (!result.emplace(key->second, std::move(data)).second)
            throw std::runtime_error("Two memory exports map to the result key " + key->second);
    }
    return result;
}
} // namespace wmtk::io
