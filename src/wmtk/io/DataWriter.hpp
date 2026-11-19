#pragma once
#include <functional>
#include <paraviewo/VTUWriter.hpp>
#include <set>
#include <stdexcept>
#include <wmtk/MeshData.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::io {
// An output observer shared by the legacy exporters and their in-memory adapters.
// File calls delegate to the original writer, retaining its formatting and filenames.
class DataWriter
{
    MeshMap* output;
    paraviewo::VTUWriter file;
    MeshAttributes points, fields;
    // The fields written, by name; the others are left out. Empty: every field. A result trims
    // its annotations so (topological_offset's), its debug frames not.
    std::function<bool(const std::string&)> kept;

public:
    explicit DataWriter(
        MeshMap* meshes = nullptr,
        std::function<bool(const std::string&)> kept_fields = {})
        : output(meshes)
        , kept(std::move(kept_fields))
    {}
    void add_field(const std::string& name, const Eigen::MatrixXd& data)
    {
        if (kept && !kept(name)) return;
        if (output)
            points[name].values = DataMatrix<double>(data);
        else
            file.add_field(name, data);
    }
    void add_integer_field(const std::string& name, const DataMatrix<int64_t>& values)
    {
        if (kept && !kept(name)) return;
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
        if (kept && !kept(name)) return;
        if (output)
            fields[name].values = DataMatrix<double>(data);
        else
            file.add_cell_field(name, data);
    }
    void add_integer_cell_field(const std::string& name, const DataMatrix<int64_t>& values)
    {
        if (kept && !kept(name)) return;
        if (output)
            fields[name].values = values;
        else {
            MeshAttribute field;
            field.values = values;
            file.add_cell_field(name, field.doubles());
        }
    }
    // A cell field in its own type, with its column labels: the memory output keeps both; the
    // file, as every field there, has Float64 and no labels.
    void add_cell_field(const std::string& name, const MeshAttribute& field)
    {
        if (kept && !kept(name)) return;
        if (output)
            fields[name] = field;
        else
            file.add_cell_field(name, field.doubles());
    }
    bool write_mesh(
        const std::string& path,
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& C,
        paraviewo::CellType type)
    {
        if (!output) return file_written(path, file.write_mesh(path, V, C, type));
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
        if (!output) return file_written(path, file.write_mesh(path, V, cells));
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
        // Each block its rows of every cell field, in the field's own type, with its labels.
        for (const auto& [type, indices] : rows)
            for (const auto& [name, a] : fields) {
                MeshAttribute& field = data.cells[type].attributes[name];
                field.labels = a.labels;
                field.values = std::visit(
                    [&indices](const auto& values) -> AttributeValues {
                        std::decay_t<decltype(values)> sub(indices.size(), values.cols());
                        for (size_t r = 0; r < indices.size(); ++r)
                            sub.row(r) = values.row(indices[r]);
                        return sub;
                    },
                    a.values);
            }
        data.validate();
        output->insert_or_assign(path, std::move(data));
        // Like VTUWriter::write_mesh, which clears its fields after every write.
        points.clear();
        fields.clear();
        return true;
    }
    // write_mesh(), and a "Write <path>" line for the file it wrote: none from a writer into
    // memory, which writes no file, nor for a file that could not be written (a warning).
    bool write_mesh_and_log(
        const std::string& path,
        const Eigen::MatrixXd& V,
        const Eigen::MatrixXi& C,
        paraviewo::CellType type)
    {
        const bool written = write_mesh(path, V, C, type);
        if (written && !output) logger().info("Write {}", path);
        return written;
    }

private:
    // paraviewo reports a file it could not open only by returning false.
    static bool file_written(const std::string& path, const bool written)
    {
        if (!written) logger().warn("Could not write {}", path);
        return written;
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
    for (auto& r : data.V_exact.rows) {
        r = remap.at(r);
        if (r < 0) throw std::runtime_error("Exact position of an inactive vertex");
    }
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
// The exact positions of the mesh's un-rounded vertices (MeshData::V_exact), in the rows of the
// memory export `data`: by vertex slot, as most exports are before compact_slot_outputs()
// renumbers V and V_exact together, or, `packed`, in get_vertices() order, as an export that
// skips the dead slots itself is. V_exact has V's three columns; a 2D mesh's third is 0.
template <typename Mesh>
void collect_exact_positions(const Mesh& mesh, MeshData& data, const bool packed = false)
{
    data.V_exact = {};
    const auto vs = mesh.get_vertices();
    for (size_t k = 0; k < vs.size(); ++k) {
        const size_t vid = vs[k].vid(mesh);
        const auto& va = mesh.m_vertex_attribute[vid];
        if (va.m_is_rounded) continue;
        auto p = va.pos();
        data.V_exact.rows.push_back(int64_t(packed ? k : vid));
        for (Eigen::Index k = 0; k < p.size(); ++k) data.V_exact.values.push_back(p[k].get_str());
        for (Eigen::Index k = p.size(); k < data.V.cols(); ++k) data.V_exact.values.push_back("0");
    }
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
// that the caller returns in another form or that the mesh already holds: the boundary
// (`_surf`), the non-manifold or feature edges (`_edge`) and the offset's front (`_off`,
// `_front`) are the mesh's tags -- its face and edge blocks, and the tet (2D: triangle) tags
// that differ across a face -- and each was a second copy of the mesh's vertices. Until 2026-11
// they were returned as `surface`, `edges` and `offset`; isotropic_remeshing, whose mesh has no
// edge block, still returns its feature edges as `edges`, and prismatic_mesh its classified
// offset faces as `offset`. The .msh among them are the meshes a history names its points by:
// <output>_history_input.msh, simwild's and tetwild's, where the surface history (and the volume
// history) starts, and tetwild's <output>_history_output.msh, the mesh before its filter, where
// the volume history ends.
using OutputKeys = std::map<std::string, std::string>;
inline OutputKeys default_output_keys()
{
    return {
        {"mesh.vtu", "mesh"},
        {"mesh_surf.vtu", ""},
        {"mesh_edge.vtu", ""},
        {"mesh_off.vtu", ""},
        {"mesh_front.vtu", ""},
        {"mesh_offset_faces.vtu", "offset"},
        {"mesh_history_input.msh", "history_input"},
        {"mesh_history_output.msh", "history_output"}};
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
