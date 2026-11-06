#pragma once
#include <cctype>
#include <filesystem>
#include <fstream>
#include <optional>
#include <set>
#include <wmtk/MeshData.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/history/History.hpp>
#include <wmtk/io/read_edge_mesh.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

namespace wmtk::io {
// Per-call state; no global mesh store, temporary files, or thread-local dispatch.
class ComponentIO
{
    detail::ComponentLoggerScope logging;
    std::optional<std::vector<uint8_t>> recorded_history, recorded_volume_history;

public:
    const MeshMap* inputs = nullptr;
    ComponentResult result{{}, nlohmann::json::object()};
    explicit ComponentIO(const MeshMap* meshes = nullptr)
        : logging(meshes != nullptr)
        , inputs(meshes)
    {}
    bool memory() const { return inputs != nullptr; }
    // A history the component recorded (wmtk/history; its history_output), of a surface or of
    // curves. The file route saves it to `path`. The memory route, which writes no file, returns
    // the bytes of that file in the report, as "history" -- a JSON binary value, bytes in Python
    // (take_result()).
    template <int N>
    void save_history(const history::History<N>& h, const std::string& path)
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
    // The volume around a tet mesh's tracked surface, recorded with its surface history
    // (volume_history_output), as the bytes of its file (history::VolumeRecorder::to_bytes). The
    // file route writes them to `path`; the memory route returns them in the report, as
    // "volume_history", next to "history".
    void save_volume_history(std::vector<uint8_t> bytes, const std::string& path)
    {
        if (!memory()) {
            history::write_history(path, bytes);
            logger().info(
                "volume history: {:.1f} MB saved to {}",
                double(bytes.size()) / 1e6,
                path);
            return;
        }
        logger().info(
            "volume history: {:.1f} MB, returned in the report",
            double(bytes.size()) / 1e6);
        recorded_volume_history = std::move(bytes);
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
    // What a memory call returns: the result, with the histories recorded, if any, in its
    // report.
    ComponentResult take_result()
    {
        if (recorded_history)
            result.second["history"] = nlohmann::json::binary(std::move(*recorded_history));
        if (recorded_volume_history) {
            result.second["volume_history"] =
                nlohmann::json::binary(std::move(*recorded_volume_history));
        }
        recorded_history.reset();
        recorded_volume_history.reset();
        return std::move(result);
    }
    const MeshData& mesh(const std::string& name) const
    {
        const auto it = inputs->find(name);
        if (it == inputs->end()) throw std::runtime_error("Unknown input mesh: " + name);
        it->second.validate();
        return it->second;
    }
    // Mesh fields come in two kinds. DATA is what a component reads: `tags`, the label columns of
    // a cell block, on every route; the envelope mesh; and the named fields of data_fields. Every
    // other field is an ANNOTATION, written for people -- tetwild's energy, triwild's quality,
    // the inputs' winding numbers wn_<name>, the offset's diagnostics -- and every reader drops
    // it: leaves it unread and names it once per read, at info (dropped_fields(), log_dropped()).
    // Until 2026-11 each reader listed the fields it took and threw `Unknown mesh attribute` on
    // any other, so a new field had to be added to every list or a chain broke: triwild's
    // wn_<name> broke triwild -> simwild on the memory route.
    //
    // The named data fields, each with who reads it; a writer that trims its result to data
    // (topological_offset's) keeps them. Besides them, a call may name fields it reads
    // (isotropic_remeshing's feature_fields), tracking's transfer carries every triangle field,
    // and simwild reads 2D label images, cell fields tag_<k> (tags_from_label_images()).
    inline static const std::set<std::string> data_fields = {
        "vid", // vertex: the ids corr_input_vid names: simwild, prismatic_mesh
        "corr_input_vid", // vertex: an offset's correspondence: simwild, prismatic_mesh
        "labels", // vertex: the offset's vertex labels: prismatic_mesh
        "is_cone", // vertex: c1_simplification
        "vertex_id", // vertex and point: isotropic_remeshing's features
        "offset_tag", // tet: the offset's band: prismatic_mesh
        "tag_0", // tet: the input region as a field: prismatic_mesh
        "edge_id", // edge: isotropic_remeshing's features
        "dofs", // triangle: c1_simplification
        "uv", // triangle, per corner: isotropic_remeshing's feature_fields, tracking
        "normal", // triangle, per corner: as uv
    };
    // The fields io::read_mesh attaches to an OBJ's triangles, UVs and normals: a value per
    // corner, as tracking's transfer and isotropic_remeshing's feature_fields read them.
    inline static const std::set<std::string> reader_corner_attributes = {"uv", "normal"};
    /// The fields of `a` a reader drops: every one but those it reads, `read`. It throws for one
    /// of `required` that `a` lacks.
    [[nodiscard]] static std::set<std::string> dropped_fields(
        const MeshAttributes& a,
        const std::set<std::string>& read,
        const std::set<std::string>& required = {})
    {
        for (const auto& name : required)
            if (!a.count(name))
                throw std::runtime_error("Missing required mesh attribute: " + name);
        std::set<std::string> dropped;
        for (const auto& [name, field] : a)
            if (!read.count(name)) dropped.insert(name);
        return dropped;
    }
    /// The fields a read dropped (dropped_fields()), named once, at info: dropping an annotation
    /// is no defect. `what` names the mesh.
    static void log_dropped(const std::string& what, const std::set<std::string>& dropped)
    {
        if (dropped.empty()) return;
        logger().info("{}: fields not read, dropped: {}", what, fmt::join(dropped, ", "));
    }
    /// What an entry of an `input` list is (tetwild, triwild and simwild), read off the input
    /// itself: triangles make a surface, segments a curve, bare vertices points. On the memory
    /// route by the mesh's blocks -- triangles, else edges, else points (a point block, or the
    /// vertices alone); on the file route an OBJ by its records -- an `f`, else an `l`, else
    /// points -- and any other format a surface, which read_triangle_mesh() reads. Until 2026-11
    /// curves and points came in lists of their own, input_edges and input_points.
    enum class InputKind { surface, curve, points };
    InputKind input_kind(const std::string& name) const
    {
        if (memory()) {
            const auto& data = mesh(name);
            const auto has = [&](const CellType type) {
                const auto it = data.cells.find(type);
                return it != data.cells.end() && it->second.connectivity.rows() > 0;
            };
            if (has(CellType::Triangle)) return InputKind::surface;
            if (has(CellType::Edge)) return InputKind::curve;
            return InputKind::points;
        }
        return file_input_kind(name);
    }
    /// input_kind() of a file.
    static InputKind file_input_kind(const std::string& name)
    {
        std::string extension = std::filesystem::path(name).extension().string();
        for (char& c : extension) c = char(std::tolower(static_cast<unsigned char>(c)));
        if (extension != ".obj") return InputKind::surface;
        std::ifstream in(name);
        if (!in) log_and_throw_error("Could not open input file {}", name);
        bool has_l = false;
        std::string line;
        while (std::getline(in, line)) {
            if (line.size() < 2 || (line[1] != ' ' && line[1] != '\t')) continue;
            if (line[0] == 'f') return InputKind::surface;
            if (line[0] == 'l') has_l = true;
        }
        return has_l ? InputKind::curve : InputKind::points;
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
    // One named input, uncleaned: the memory twin of reading one file. Its fields are dropped
    // (dropped_fields()) but `triangle_attributes`, which the component reads itself
    // (isotropic_remeshing's feature_fields).
    void raw_triangles(
        const std::string& name,
        MatrixXd& V,
        MatrixXi& F,
        const std::set<std::string>& triangle_attributes = {}) const
    {
        const auto& data = mesh(name);
        std::set<std::string> dropped = dropped_fields(data.vertex_attributes, {});
        for (const auto& [type, c] : data.cells) {
            if (type != CellType::Triangle)
                throw std::runtime_error("Expected triangle input: " + name);
            dropped.merge(dropped_fields(c.attributes, triangle_attributes));
        }
        log_dropped("input " + name, dropped);
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
    // `read`, as there: the inputs as given, concatenated, and where their vertices and faces
    // went. `triangle_attributes`, as in raw_triangles.
    void triangles(
        const std::vector<std::string>& names,
        MatrixXd& V,
        MatrixXi& F,
        double rel,
        double abs,
        std::vector<int>* provenance = nullptr,
        ReadProvenance* read = nullptr,
        const std::set<std::string>& triangle_attributes = {}) const
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
            raw_triangles(names[i], v, f, triangle_attributes);
            const auto nv = V.rows(), nf = F.rows();
            f.array() += nv;
            V.conservativeResize(nv + v.rows(), 3);
            V.bottomRows(v.rows()) = v;
            F.conservativeResize(nf + f.rows(), 3);
            F.bottomRows(f.rows()) = f;
            if (provenance) provenance->resize(F.rows(), int(i));
        }
        clean_triangle_mesh(V, F, rel, abs, provenance, read);
        if (!F.rows()) throw std::runtime_error("No faces after cleaning");
    }
    // `read`, as read_edge_mesh's provenance: the input as given, and where its vertices and
    // segments went -- filled by the cleaning, as for triangles.
    void edges(
        const std::string& name,
        MatrixXd& V,
        MatrixXi& E,
        double rel = -1,
        double abs = -1,
        ReadProvenance* read = nullptr) const
    {
        if (!memory()) {
            read_edge_mesh(name, V, E, rel, abs, read);
            return;
        }
        const auto& data = mesh(name);
        std::set<std::string> dropped = dropped_fields(data.vertex_attributes, {});
        for (const auto& [type, c] : data.cells) {
            if (type != CellType::Edge && type != CellType::Point)
                throw std::runtime_error("Expected curves or points: " + name);
            dropped.merge(dropped_fields(c.attributes, {}));
        }
        log_dropped("input " + name, dropped);
        V = data.V;
        E.resize(0, 2);
        auto it = data.cells.find(CellType::Edge);
        if (it != data.cells.end()) E = indices(it->second.connectivity);
        clean_edge_mesh(V, E, rel, abs, read);
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
