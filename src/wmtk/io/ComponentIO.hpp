#pragma once
#include <optional>
#include <set>
#include <wmtk/MeshData.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/history/SurfaceHistory.hpp>
#include <wmtk/io/read_edge_mesh.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

namespace wmtk::io {
// Per-call state; no global mesh store, temporary files, or thread-local dispatch.
class ComponentIO
{
    detail::ComponentLoggerScope logging;
    std::optional<std::vector<uint8_t>> recorded_history;

public:
    const MeshMap* inputs = nullptr;
    ComponentResult result{{}, nlohmann::json::object()};
    explicit ComponentIO(const MeshMap* meshes = nullptr)
        : logging(meshes != nullptr)
        , inputs(meshes)
    {}
    bool memory() const { return inputs != nullptr; }
    // A history the component recorded (wmtk/history; its history_output). The file route saves
    // it to `path`. The memory route, which writes no file, returns the bytes of that file in
    // the report, as "history" -- a JSON binary value, bytes in Python (take_result()).
    void save_history(const history::SurfaceHistory& h, const std::string& path)
    {
        if (!memory()) {
            h.save(path);
            logger().info("history: {} edits saved to {}", h.edits.size(), path);
            return;
        }
        recorded_history = h.to_bytes();
        logger().info(
            "history: {} edits, {} bytes, returned in the report",
            h.edits.size(),
            recorded_history->size());
    }
    // The metadata of a history the component records: its parameters, and the route it ran on,
    // "file" or "memory". On the memory route the inputs the parameters name are meshes of the
    // call, which no file holds.
    std::string history_metadata(const nlohmann::json& params) const
    {
        nlohmann::json metadata = params;
        metadata["route"] = memory() ? "memory" : "file";
        return metadata.dump();
    }
    // What a memory call returns: the result, with the history recorded, if any, in its report.
    ComponentResult take_result()
    {
        if (recorded_history)
            result.second["history"] = nlohmann::json::binary(std::move(*recorded_history));
        recorded_history.reset();
        return std::move(result);
    }
    const MeshData& mesh(const std::string& name) const
    {
        const auto it = inputs->find(name);
        if (it == inputs->end()) throw std::runtime_error("Unknown input mesh: " + name);
        it->second.validate();
        return it->second;
    }
    // Fields io::read_mesh attaches to plain geometry files: MSH source IDs and OBJ per-corner
    // UVs/normals. Geometry-only inputs carry no attributes through the algorithm, so these are
    // ignored, as the file readers ignore them; any other attribute is still rejected.
    inline static const std::set<std::string> reader_vertex_attributes = {"vid"};
    inline static const std::set<std::string> reader_corner_attributes = {"uv", "normal"};
    static void attributes(
        const MeshAttributes& a,
        const std::set<std::string>& allowed,
        const std::set<std::string>& required = {})
    {
        for (const auto& name : required)
            if (!a.count(name))
                throw std::runtime_error("Missing required mesh attribute: " + name);
        for (const auto& [name, field] : a)
            if (!allowed.count(name)) throw std::runtime_error("Unknown mesh attribute: " + name);
    }
    std::string path(const std::filesystem::path& root, const nlohmann::json& selector) const
    {
        return memory() ? selector.get<std::string>()
                        : utils::resolve_path(root, selector.get<std::string>()).string();
    }
    std::vector<std::string> paths(
        const std::filesystem::path& root,
        const nlohmann::json& selectors) const
    {
        if (selectors.is_string()) return {path(root, selectors)};
        std::vector<std::string> out;
        for (const auto& v : selectors) out.push_back(path(root, v));
        return out;
    }
    // One named input, uncleaned: the memory twin of reading one file.
    void raw_triangles(const std::string& name, MatrixXd& V, MatrixXi& F) const
    {
        const auto& data = mesh(name);
        attributes(data.vertex_attributes, reader_vertex_attributes);
        for (const auto& [type, c] : data.cells) {
            if (type != CellType::Triangle)
                throw std::runtime_error("Expected triangle input: " + name);
            attributes(c.attributes, reader_corner_attributes);
        }
        V = data.V;
        F = indices(data.cell(CellType::Triangle).connectivity);
        if (!F.rows()) throw std::runtime_error("Empty triangle input: " + name);
    }
    void triangles(
        const std::string& name,
        MatrixXd& V,
        MatrixXi& F,
        double rel = -1,
        double abs = -1) const
    {
        if (!memory()) {
            read_triangle_mesh(name, V, F, rel, abs);
            return;
        }
        raw_triangles(name, V, F);
        clean_triangle_mesh(V, F, rel, abs);
        if (!F.rows()) throw std::runtime_error("No faces after cleaning: " + name);
    }
    // Like read_triangle_mesh(paths): concatenate the raw inputs, then clean them once, so
    // vertex merging sees the same vertices (isolated ones included) in the same order.
    // `read`, as there: the inputs as given, concatenated, and where their vertices went.
    void triangles(
        const std::vector<std::string>& names,
        MatrixXd& V,
        MatrixXi& F,
        double rel,
        double abs,
        std::vector<int>* provenance = nullptr,
        ReadProvenance* read = nullptr) const
    {
        if (!memory()) {
            read_triangle_mesh(names, V, F, rel, abs, provenance, read);
            return;
        }
        V.resize(0, 3);
        F.resize(0, 3);
        if (provenance) provenance->clear();
        for (size_t i = 0; i < names.size(); ++i) {
            MatrixXd v;
            MatrixXi f;
            raw_triangles(names[i], v, f);
            const auto nv = V.rows(), nf = F.rows();
            f.array() += nv;
            V.conservativeResize(nv + v.rows(), 3);
            V.bottomRows(v.rows()) = v;
            F.conservativeResize(nf + f.rows(), 3);
            F.bottomRows(f.rows()) = f;
            if (provenance) provenance->resize(F.rows(), int(i));
        }
        if (read) {
            read->V = V;
            read->F = F;
        }
        clean_triangle_mesh(V, F, rel, abs, provenance, read ? &read->vertex_map : nullptr);
        if (!F.rows()) throw std::runtime_error("No faces after cleaning");
    }
    void edges(const std::string& name, MatrixXd& V, MatrixXi& E, double rel = -1, double abs = -1)
        const
    {
        if (!memory()) {
            read_edge_mesh(name, V, E, rel, abs);
            return;
        }
        const auto& data = mesh(name);
        attributes(data.vertex_attributes, reader_vertex_attributes);
        for (const auto& [type, c] : data.cells) {
            if (type != CellType::Edge && type != CellType::Point)
                throw std::runtime_error("Expected curves or points: " + name);
            attributes(c.attributes, {});
        }
        V = data.V;
        E.resize(0, 2);
        auto it = data.cells.find(CellType::Edge);
        if (it != data.cells.end()) E = indices(it->second.connectivity);
        clean_edge_mesh(V, E, rel, abs);
    }
    static MatrixXi indices(const DataMatrix<int64_t>& c)
    {
        if (c.size() && (c.maxCoeff() > std::numeric_limits<int>::max() || c.minCoeff() < 0))
            throw std::runtime_error("Connectivity exceeds internal index range");
        return c.cast<int>();
    }
    void prepare(nlohmann::json& j, const std::string& component) const
    {
        if (!memory()) return;
        if ((component == "simwild" || component == "tetwild" || component == "triwild") &&
            j.contains("input") && j["input"].is_string())
            j["input"] = nlohmann::json::array({j["input"]});
        j["application"] = component;
        j["input_dir"] = "";
        j["output"] = "";
        j["log_file"] = "";
        j["report"] = "";
    }
};

template <typename Mesh, typename Position>
MeshData triangle_data(const Mesh& m, Position position)
{
    MeshData data;
    const auto vs = m.get_vertices();
    const auto fs = m.get_faces();
    data.V.resize(vs.size(), 3);
    std::map<size_t, int64_t> rows;
    for (size_t i = 0; i < vs.size(); ++i) {
        rows[vs[i].vid(m)] = i;
        data.V.row(i) = position(vs[i].vid(m));
    }
    auto& c = data.cells[CellType::Triangle].connectivity;
    c.resize(fs.size(), 3);
    for (size_t i = 0; i < fs.size(); ++i) {
        auto v = m.oriented_tri_vertices(fs[i]);
        for (int j = 0; j < 3; ++j) c(i, j) = rows.at(v[j].vid(m));
    }
    data.validate();
    return data;
}
} // namespace wmtk::io
