#include "isotropic_remeshing.hpp"
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/utils/Preallocation.hpp>

#include <igl/Timer.h>
#include <jse/jse.h>

#include <wmtk/utils/getRSS.h>
#include <wmtk/Types.hpp>
#include <wmtk/history/InputHistory.hpp>
#include <wmtk/io/DataWriter.hpp>
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>
#include <wmtk/utils/io.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <set>

#include <isotropic_remeshing_spec.hpp>
#include "IsotropicRemeshing.h"

namespace wmtk::components::isotropic_remeshing {

namespace {

/// What field_boundaries reads from a mesh.
struct FieldData
{
    /// The edges between two faces that differ in any of the fields.
    std::vector<std::array<size_t, 2>> boundaries;
    /// The per-face fields among them, in the order given, with their number of components.
    std::vector<std::pair<std::string, size_t>> face_fields;
    /// Each of face_fields as given, without its rows: its type and its column labels, which the
    /// memory route returns it with. Each type reaches face_values exactly: Float64 as it is,
    /// UInt8, and Int64 within 2^53, beyond which field_boundaries refuses it.
    std::vector<MeshAttribute> face_attributes;
    /// Each face's values of face_fields, one after the other.
    std::vector<std::vector<double>> face_values;
};

/// The per-corner fields io::read_mesh reads (io::ComponentIO::reader_corner_attributes), with
/// their number of columns: corner after corner, two values per corner for uv, three for normal.
const std::map<std::string, Eigen::Index> corner_columns = {{"uv", 6}, {"normal", 9}};

/// The edges between two faces that differ in any of `fields`, triangle fields of `mesh` -- the
/// mesh file as io::read_mesh reads it, or the memory input -- of whose triangles the reader made
/// `tris`: `face_map` (io::ReadProvenance::cell_map) takes each of them to the one of `tris` it
/// became, or is -1 for a degenerate one, which the cleaning dropped, and its values with it.
/// `path` names it in the errors. A per-corner field (see corner_columns) differs across an edge
/// where the two faces disagree on it at either end: a texture seam, say.
FieldData field_boundaries(
    const MeshData& mesh,
    const std::string& path,
    const std::vector<int>& face_map,
    const std::vector<std::array<size_t, 3>>& tris,
    const std::vector<std::string>& fields)
{
    const auto it = mesh.cells.find(CellType::Triangle);
    const Eigen::Index n_faces = it == mesh.cells.end() ? 0 : it->second.connectivity.rows();
    if (size_t(n_faces) != face_map.size()) {
        log_and_throw_error(
            "feature_fields: {} has {} triangles as io::read_mesh reads it and {} as the "
            "remeshing reads it, so its fields do not line up with them",
            path,
            n_faces,
            face_map.size());
    }
    FieldData out;
    std::vector<std::vector<double>>& value = out.face_values;
    value.resize(tris.size());
    std::vector<std::array<std::vector<double>, 3>> corner_value(tris.size());
    for (const std::string& name : fields) {
        const auto a = it->second.attributes.find(name);
        if (a == it->second.attributes.end()) {
            log_and_throw_error("feature_fields: {} has no field {}", path, name);
        }
        // The remeshing carries the values as Float64, which holds every integer up to 2^53 in
        // magnitude and only some beyond: the limit is 2^53, not whichever ids happen to fit.
        if (const auto* ints = std::get_if<DataMatrix<int64_t>>(&a->second.values)) {
            constexpr int64_t limit = int64_t(1) << 53;
            for (Eigen::Index i = 0; i < ints->size(); ++i) {
                const int64_t x = ints->data()[i];
                if (x > limit || x < -limit) {
                    log_and_throw_error(
                        "feature_fields: {} in {} has the value {}, beyond 2^53 in magnitude: "
                        "the remeshing carries it as Float64, which holds integers exactly only "
                        "up to 2^53",
                        name,
                        path,
                        x);
                }
            }
        }
        const DataMatrix<double> values = a->second.doubles();
        const auto columns = corner_columns.find(name);
        const bool corner = columns != corner_columns.end();
        // Known by its name alone, as io::read_mesh names it: a per-face normal of 3 columns, say,
        // would be read as one value per corner.
        if (corner && values.cols() != columns->second) {
            log_and_throw_error(
                "feature_fields: {0} in {1} has {2} columns, but {0} is per corner, as "
                "io::read_mesh reads it: {3} columns, {4} values per corner, corner after corner",
                name,
                path,
                values.cols(),
                columns->second,
                columns->second / 3);
        }
        if (!corner) {
            out.face_fields.emplace_back(name, size_t(values.cols()));
            MeshAttribute given;
            given.labels = a->second.labels;
            given.values = std::visit(
                [](const auto& m) -> AttributeValues {
                    return std::decay_t<decltype(m)>(0, m.cols());
                },
                a->second.values);
            out.face_attributes.push_back(std::move(given));
        }
        const Eigen::Index k = values.cols() / 3;
        for (size_t i = 0; i < face_map.size(); ++i) {
            if (face_map[i] < 0) continue; // gone, and its values with it
            const size_t f = size_t(face_map[i]);
            const double* row = values.row(i).data();
            if (!corner) {
                value[f].insert(value[f].end(), row, row + values.cols());
                continue;
            }
            for (int c = 0; c < 3; ++c) {
                corner_value[f][c].insert(corner_value[f][c].end(), row + c * k, row + (c + 1) * k);
            }
        }
    }
    const auto at = [&](size_t f, size_t v) -> const std::vector<double>& {
        for (int c = 0; c < 3; ++c) {
            if (tris[f][c] == v) return corner_value[f][c];
        }
        return corner_value[f][0];
    };
    std::map<std::array<size_t, 2>, std::vector<size_t>> edge_faces;
    for (size_t f = 0; f < tris.size(); ++f) {
        for (int k = 0; k < 3; ++k) {
            size_t a = tris[f][k], b = tris[f][(k + 1) % 3];
            if (a > b) std::swap(a, b);
            edge_faces[{{a, b}}].push_back(f);
        }
    }
    for (const auto& [e, fs] : edge_faces) {
        if (fs.size() != 2) continue;
        const size_t f = fs[0], g = fs[1];
        if (value[f] != value[g] || at(f, e[0]) != at(g, e[0]) || at(f, e[1]) != at(g, e[1])) {
            out.boundaries.push_back(e);
        }
    }
    return out;
}

/// Curves with ids and fixed vertices with theirs (see the spec's features).
struct Features
{
    std::vector<std::array<size_t, 2>> edges;
    std::vector<int64_t> edge_ids;
    std::vector<std::pair<size_t, int64_t>> vertices;
};

/// The file route's features: a features file, as it is, its indices vertices of the input as
/// given, of which there are `n_vertices`.
Features read_features(const std::string& path, const size_t n_vertices)
{
    nlohmann::json j;
    {
        std::ifstream in(path);
        if (!in) log_and_throw_error("features: cannot open {}", path);
        try {
            j = nlohmann::json::parse(in);
        } catch (const std::exception& e) {
            log_and_throw_error("features: {} is not JSON: {}", path, e.what());
        }
    }
    if (!j.is_object()) {
        log_and_throw_error("features: {} is not an object of edges, edge_ids and vertices", path);
    }
    const auto list = [&](const char* key) {
        const nlohmann::json v = j.value(key, nlohmann::json::array());
        if (!v.is_array()) log_and_throw_error("features: {} in {} is not an array", key, path);
        return v;
    };
    // An integer, not a number that would be truncated to one.
    const auto id = [&](const nlohmann::json& v, const char* what) -> int64_t {
        if (!v.is_number_integer()) {
            log_and_throw_error("features: {} {} in {} is not an integer", what, v.dump(), path);
        }
        return v.get<int64_t>();
    };
    const auto vertex = [&](const nlohmann::json& v, const char* what) -> size_t {
        if (!v.is_number_integer() || v.get<int64_t>() < 0 || v.get<size_t>() >= n_vertices) {
            log_and_throw_error(
                "features: {} {} in {} is not a vertex of the input ({} vertices)",
                what,
                v.dump(),
                path,
                n_vertices);
        }
        return v.get<size_t>();
    };

    Features out;
    const nlohmann::json edges = list("edges");
    const nlohmann::json ids = list("edge_ids");
    if (!ids.empty() && ids.size() != edges.size()) {
        log_and_throw_error(
            "features: {} has {} edge_ids for {} edges",
            path,
            ids.size(),
            edges.size());
    }
    for (size_t i = 0; i < edges.size(); ++i) {
        if (!edges[i].is_array() || edges[i].size() != 2) {
            log_and_throw_error("features: edge {} in {} is not a pair", edges[i].dump(), path);
        }
        out.edges.push_back({{vertex(edges[i][0], "edge end"), vertex(edges[i][1], "edge end")}});
        out.edge_ids.push_back(ids.empty() ? -1 : id(ids[i], "edge id"));
    }
    for (const nlohmann::json& v : list("vertices")) {
        if (!v.is_array() || v.size() != 2) {
            log_and_throw_error("features: vertex {} in {} is not [vertex, id]", v.dump(), path);
        }
        out.vertices.emplace_back(vertex(v[0], "vertex"), id(v[1], "vertex id"));
    }
    return out;
}

/// The vertex fields a memory input may carry: those of io::read_mesh, and the vertex_id of a
/// result, so that a result can be passed back in.
std::set<std::string> with_vertex_id()
{
    std::set<std::string> names = io::ComponentIO::reader_vertex_attributes;
    names.insert("vertex_id");
    return names;
}

/// The memory route's features: the mesh `name`, `data`, on the vertex rows of the input as given
/// (`n_vertices` of them) as the edges result is on mesh's, so that one call's result can be the
/// next one's features. Its edges are the curves, with the Int64 cell field edge_id or without;
/// the fixed vertices are its points, with the Int64 cell field vertex_id, and the vertices whose
/// Int64 vertex field vertex_id is not -1, as mesh and edges carry them.
Features read_features(const MeshData& data, const std::string& name, const size_t n_vertices)
{
    const auto refuse = [&name](const std::string& what) {
        log_and_throw_error("features: the mesh {} {}", name, what);
    };
    io::ComponentIO::attributes(data.vertex_attributes, with_vertex_id());
    for (const auto& [type, c] : data.cells) {
        if (type == CellType::Edge) {
            io::ComponentIO::attributes(c.attributes, {"edge_id"});
        } else if (type == CellType::Point) {
            io::ComponentIO::attributes(c.attributes, {"vertex_id"}, {"vertex_id"});
        } else {
            refuse("has cells other than edges and points");
        }
    }
    if (size_t(data.V.rows()) != n_vertices) {
        refuse(
            fmt::format(
                "has {} vertices and the input {}: its indices are rows of the input",
                data.V.rows(),
                n_vertices));
    }
    // One Int64 column: an id never goes through Float64.
    const auto ids = [&](const MeshAttributes& a, const std::string& field, Eigen::Index rows) {
        const auto it = a.find(field);
        if (it == a.end()) return DataMatrix<int64_t>(DataMatrix<int64_t>::Constant(rows, 1, -1));
        const auto* values = std::get_if<DataMatrix<int64_t>>(&it->second.values);
        if (!values || values->cols() != 1)
            refuse("has a " + field + " that is not one Int64 column");
        return *values;
    };
    Features out;
    if (const auto e = data.cells.find(CellType::Edge); e != data.cells.end()) {
        const auto& c = e->second.connectivity;
        const auto id = ids(e->second.attributes, "edge_id", c.rows());
        for (Eigen::Index i = 0; i < c.rows(); ++i) {
            out.edges.push_back({{size_t(c(i, 0)), size_t(c(i, 1))}});
            out.edge_ids.push_back(id(i, 0));
        }
    }
    if (const auto p = data.cells.find(CellType::Point); p != data.cells.end()) {
        const auto& c = p->second.connectivity;
        const auto id = ids(p->second.attributes, "vertex_id", c.rows());
        for (Eigen::Index i = 0; i < c.rows(); ++i)
            out.vertices.emplace_back(size_t(c(i, 0)), id(i, 0));
    }
    if (data.vertex_attributes.count("vertex_id")) {
        const auto id = ids(data.vertex_attributes, "vertex_id", data.V.rows());
        for (Eigen::Index v = 0; v < id.rows(); ++v) {
            if (id(v, 0) != -1) out.vertices.emplace_back(size_t(v), id(v, 0));
        }
    }
    return out;
}

/// The features `in`, of the input as given, on the mesh the reader made of it: `vertex_map`
/// takes each vertex of the input to the one it became, or -1 (see io::ReadProvenance), and
/// `tris` are the triangles it made. An edge whose two ends became one goes: a B-rep edge of
/// no length, or one whose ends were duplicates. Entries that became one, of edges or of
/// vertices, are kept once, and have to agree on the id. Each edge has to be an edge of `tris`,
/// and each vertex a vertex of them. `source` names the features in the errors.
Features weld_features(
    const Features& in,
    const std::vector<int>& vertex_map,
    const std::vector<std::array<size_t, 3>>& tris,
    const std::string& source)
{
    std::set<std::array<size_t, 2>> mesh_edges;
    for (const auto& t : tris) {
        for (int k = 0; k < 3; ++k) {
            mesh_edges.insert({{std::min(t[k], t[(k + 1) % 3]), std::max(t[k], t[(k + 1) % 3])}});
        }
    }
    Features out;
    // Each kept entry, with its id and the entry of `in` it came from first.
    std::map<std::array<size_t, 2>, std::pair<int64_t, std::array<size_t, 2>>> seen;
    for (size_t i = 0; i < in.edges.size(); ++i) {
        const auto [a, b] = in.edges[i];
        const int64_t id = in.edge_ids[i];
        const int x = vertex_map.at(a), y = vertex_map.at(b);
        if (x == y && (x >= 0 || a == b)) continue; // no length: its one vertex, no mesh edge
        const std::array<size_t, 2> e{{size_t(std::min(x, y)), size_t(std::max(x, y))}};
        if (x < 0 || y < 0 || !mesh_edges.count(e)) {
            log_and_throw_error(
                "features: ({}, {}) in {} is not an edge of the input",
                a,
                b,
                source);
        }
        const auto [it, inserted] = seen.emplace(e, std::pair{id, in.edges[i]});
        if (!inserted) {
            const auto [first_id, first] = it->second;
            if (first_id != id) {
                log_and_throw_error(
                    "features: ({}, {}) and ({}, {}) in {} are one edge of the input, with ids {} "
                    "and {}",
                    first[0],
                    first[1],
                    a,
                    b,
                    source,
                    first_id,
                    id);
            }
            continue;
        }
        out.edges.push_back(e);
        out.edge_ids.push_back(id);
    }
    std::map<size_t, std::pair<int64_t, size_t>> fixed;
    for (const auto& [v, id] : in.vertices) {
        const int x = vertex_map.at(v);
        if (x < 0) {
            log_and_throw_error(
                "features: vertex {} in {} is in no triangle of the input",
                v,
                source);
        }
        const auto [it, inserted] = fixed.emplace(size_t(x), std::pair{id, v});
        if (!inserted && it->second.first != id) {
            log_and_throw_error(
                "features: vertices {} and {} in {} are one vertex of the input, with ids {} and "
                "{}",
                it->second.second,
                v,
                source,
                it->second.first,
                id);
        }
        if (inserted) out.vertices.emplace_back(size_t(x), id);
    }
    return out;
}

/// Writes the consolidated mesh to `path` (.vtu): its triangles with a cell field per per-face
/// field and the point field vertex_id, the id of a fixed vertex and -1 elsewhere; and its
/// feature edges to <stem>_edge.vtu, as simwild names its own, on the same points, a line cell
/// each with the cell field edge_id (-1 for none) and the same vertex_id. With `memory`, the
/// memory route's: the two meshes go there instead, under the names of the files (see
/// io::DataWriter), the ids Int64 -- the .vtu has Float64 and no labels, as every field there --
/// the per-face fields in the type they were given in, with their labels, and the edges with an
/// edge block even where there are none.
void write_vtu(
    const IsotropicRemeshing& m,
    const std::filesystem::path& path,
    const FieldData& fields,
    MeshMap* memory = nullptr)
{
    const auto verts = m.get_vertices();
    std::vector<int> index(m.vert_capacity(), -1);
    Eigen::MatrixXd V(verts.size(), 3);
    DataMatrix<int64_t> vertex_id(verts.size(), 1);
    for (size_t i = 0; i < verts.size(); ++i) {
        const size_t v = verts[i].vid(m);
        index[v] = int(i);
        V.row(i) = m.vertex_attrs.at(v).pos;
        vertex_id(i, 0) = m.is_fixed(v) ? m.feature_attrs.at(v).vertex_id : -1;
    }

    const auto faces = m.get_faces();
    Eigen::MatrixXi F(faces.size(), 3);
    for (size_t i = 0; i < faces.size(); ++i) {
        const auto vs = m.oriented_tri_vids(faces[i]);
        for (int k = 0; k < 3; ++k) F(i, k) = index[vs[k]];
    }
    io::DataWriter triangles(memory);
    triangles.add_integer_field("vertex_id", vertex_id);
    size_t offset = 0;
    for (size_t j = 0; j < fields.face_fields.size(); ++j) {
        const auto& [name, components] = fields.face_fields[j];
        Eigen::MatrixXd values(faces.size(), components);
        for (size_t i = 0; i < faces.size(); ++i) {
            const auto& f = m.face_field_attrs.at(faces[i].fid(m)).values;
            for (size_t c = 0; c < components; ++c) values(i, c) = f.at(offset + c);
        }
        // Back to the type it was given in, exactly (see FieldData::face_attributes).
        MeshAttribute field = fields.face_attributes[j];
        field.values = std::visit(
            [&values](const auto& given) -> AttributeValues {
                using Scalar = typename std::decay_t<decltype(given)>::Scalar;
                return DataMatrix<Scalar>(values.cast<Scalar>());
            },
            field.values);
        triangles.add_cell_field(name, field);
        offset += components;
    }
    if (!triangles.write_mesh(path.string(), V, F, paraviewo::CellType::Triangle)) {
        log_and_throw_error("Could not write {}", path.string());
    }

    const auto features = m.feature_edges();
    Eigen::MatrixXi E(features.size(), 2);
    DataMatrix<int64_t> edge_id(features.size(), 1);
    for (size_t i = 0; i < features.size(); ++i) {
        E(i, 0) = index[features[i][0]];
        E(i, 1) = index[features[i][1]];
        edge_id(i, 0) = m.feature_edge_id(features[i][0], features[i][1]);
    }
    std::filesystem::path edges_path = path;
    edges_path.replace_filename(path.stem().string() + "_edge.vtu");
    io::DataWriter lines(memory);
    lines.add_integer_field("vertex_id", vertex_id);
    lines.add_integer_cell_field("edge_id", edge_id);
    if (!lines.write_mesh(edges_path.string(), V, E, paraviewo::CellType::Line)) {
        log_and_throw_error("Could not write {}", edges_path.string());
    }
    if (memory) {
        // The exporter makes only the blocks it fills; fixed vertices without curves still have
        // edges, none of them.
        CellData& block = memory->at(edges_path.string()).cells[CellType::Edge];
        if (block.connectivity.cols() != 2) {
            block.connectivity.resize(0, 2);
            block.attributes["edge_id"].values = DataMatrix<int64_t>(0, 1);
        }
        return;
    }
    logger().info(
        "wrote {} and {}: {} triangles, {} feature edges",
        path.string(),
        edges_path.string(),
        faces.size(),
        features.size());
}

} // namespace

double run_remeshing(
    std::string input,
    double len,
    std::string output,
    IsotropicRemeshing& m,
    int itrs,
    const std::function<void()>& write_output = {})
{
    igl::Timer timer;
    wmtk::logger().info("target len: {}", len);
    timer.start();
    m.uniform_remeshing(len, itrs);
    timer.stop();

    m.consolidate_mesh();
    if (write_output) {
        write_output();
    } else if (!output.empty()) {
        m.write_triangle_mesh(output);
    }
    auto properties = m.average_len_valen();
    wmtk::logger().info("runtime {:.4}s", timer.getElapsedTimeInSec());
    wmtk::logger().info("current_memory {}", getCurrentRSS() / (1024. * 1024));
    wmtk::logger().info("peak_memory {}", getPeakRSS() / (1024. * 1024));
    wmtk::logger().info("after remesh properties: {}", properties);
    wmtk::logger().info(
        "After_vertices#: {} \n\t After_tris#: {}",
        m.vert_capacity(),
        m.tri_capacity());

    return timer.getElapsedTimeInSec();
}

static void run(nlohmann::json json_params, io::ComponentIO& component_io)
{
    component_io.prepare(json_params, "isotropic_remeshing");
    using wmtk::utils::resolve_path;
    using Tuple = IsotropicRemeshing::Tuple;

    // verify input and inject defaults
    {
        const auto spec =
            jse::embed::wmtk_isotropic_remeshing_spec::isotropic_remeshing_spec::spec();
        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(json_params, spec);
        if (!r) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, spec);
    }

    const std::filesystem::path root = json_params["input_dir"];

    // logger settings
    {
        std::string log_file_name = json_params["log_file"];
        if (!log_file_name.empty()) {
            log_file_name = resolve_path(root, log_file_name).string();
            wmtk::set_file_logger(log_file_name);
            logger().flush_on(spdlog::level::info);
        }
    }

    const std::string input_path = component_io.path(root, json_params["input"]);
    const std::string output = json_params["output"];
    const double env_rel = json_params["eps_rel"];
    const double boundary_env_rel = json_params["boundary_eps_rel"];
    const double length_rel = json_params["length_rel"];
    const int num_threads = json_params["num_threads"];
    const int itrs = json_params["max_iterations"];
    const bool sample_envelope = json_params["use_sample_envelope"];
    const bool freeze_boundary = json_params["freeze_boundary"];
    const bool use_link_condition = json_params["use_link_condition"];
    const bool check_inversions = json_params["check_inversions"];
    const bool swap_quality = json_params["swap_quality"];
    const bool limit_collapse_length = json_params["limit_collapse_length"];
    const double cleanup_angle = json_params["cleanup_angle"];
    const double remove_duplicate_eps = json_params["remove_duplicate_eps"];
    const std::string history_output = json_params["history_output"];
    // Triangle fields of the input file on the file route, of the input mesh on the memory route.
    const std::vector<std::string> feature_fields = json_params["feature_fields"];
    // A file on the file route, a mesh of the memory input on the memory route.
    const std::string features_input = json_params["features"];
    double length_abs = json_params["length_abs"];
    // With feature curves, a .vtu output holds them, the fields and the fixed vertices.
    const bool vtu_output = (!feature_fields.empty() || !features_input.empty()) &&
                            std::filesystem::path(output).extension() == ".vtu";
    if (!vtu_output && !output.empty() && (!feature_fields.empty() || !features_input.empty())) {
        logger().warn(
            "{} holds the triangles only: the feature edges, their ids, the fixed vertices' ids "
            "and the per-face fields go to a .vtu output alone",
            output);
    }

    wmtk::logger().info("remeshing on {}", input_path);
    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;
    std::vector<size_t> modified_nonmanifold_v;
    // With a history: the file as read and the vertex of the mesh each of its vertices became,
    // so that the history starts at the file itself. With features too, which name vertices of
    // the file; and with feature_fields, which give values to its faces, of which the cleaning
    // drops the degenerate ones.
    wmtk::io::ReadProvenance read;
    // Where the features come from, for the messages.
    const std::string features_source = features_input.empty() ? ""
                                        : component_io.memory()
                                            ? "the mesh " + features_input
                                            : resolve_path(root, features_input).string();
    Features features;
    {
        MatrixXd V;
        MatrixXi F;
        component_io.triangles(
            std::vector<std::string>{input_path},
            V,
            F,
            remove_duplicate_eps,
            -1,
            nullptr,
            history_output.empty() && features_input.empty() && feature_fields.empty() ? nullptr
                                                                                       : &read,
            // Ignored: the fixed vertices come from features.
            with_vertex_id(),
            // Read below, by field_boundaries.
            std::set<std::string>(feature_fields.begin(), feature_fields.end()));
        box_minmax.first = V.colwise().minCoeff();
        box_minmax.second = V.colwise().maxCoeff();
        VF_to_vectors(V, F, verts, tris);
    }
    // Read here, before the mesh and its envelope are built, so that a field the remeshing cannot
    // take is refused at once.
    FieldData fields;
    if (!feature_fields.empty()) {
        if (component_io.memory()) {
            fields = field_boundaries(
                component_io.mesh(input_path),
                "the mesh " + input_path,
                read.cell_map,
                tris,
                feature_fields);
        } else {
            MeshData mesh;
            try {
                mesh = io::read_mesh(input_path);
            } catch (const std::exception& e) {
                log_and_throw_error("feature_fields: could not read {}: {}", input_path, e.what());
            }
            fields = field_boundaries(mesh, input_path, read.cell_map, tris, feature_fields);
        }
    }
    if (!features_input.empty()) {
        // Merging duplicates and dropping unreferenced vertices renumbers them; the features
        // follow them.
        const size_t n = read.vertex_map.size();
        features = weld_features(
            component_io.memory()
                ? read_features(component_io.mesh(features_input), features_input, n)
                : read_features(features_source, n),
            read.vertex_map,
            tris,
            features_source);
    }

    double diag = (box_minmax.first - box_minmax.second).norm();
    const double envelope_size = env_rel * diag;
    const double boundary_envelope_size = boundary_env_rel > 0 ? boundary_env_rel * diag : 0;
    if (boundary_envelope_size > 0) {
        wmtk::logger().info("boundary envelope eps {:.6}", boundary_envelope_size);
    } else {
        wmtk::logger().info("freeze bnd {}", freeze_boundary);
    }
    igl::Timer timer;

    IsotropicRemeshing m(verts, num_threads, !sample_envelope);
    wmtk::set_preallocation_factor_from_json(m, json_params);
    m.set_use_link_condition(use_link_condition);
    m.set_check_inversions(check_inversions);
    m.set_swap_quality(swap_quality);
    m.set_limit_collapse_length(limit_collapse_length);
    m.set_cleanup_angle(cleanup_angle); // before create_mesh: it keeps the input for the repairs
    m.create_mesh(
        verts.size(),
        tris,
        modified_nonmanifold_v,
        freeze_boundary,
        envelope_size,
        boundary_envelope_size);

    {
        size_t nm_e = 0;
        for (const Tuple& e : m.get_edges()) {
            if (m.edge_valence(e) > 2) {
                ++nm_e;
            }
        }
        size_t nm_v = 0;
        for (const Tuple& t : m.get_vertices()) {
            if (!m.is_manifold_vertex(t.vid(m))) {
                ++nm_v;
            }
        }
        logger().info(
            "Input non-manifold: {} edges, {} vertices. Link condition {}.",
            nm_e,
            nm_v,
            use_link_condition ? "on" : "off");
    }

    {
        std::vector<double> properties = m.average_len_valen();
        wmtk::logger().info("before remesh properties: {}", properties);
        if (length_abs < 0 && length_rel < 0) {
            logger().info("Use average edge length as target length.");
            length_abs = properties[0];
        } else if (length_abs < 0) {
            length_abs = diag * length_rel;
        }
    }
    logger().info("absolute target length: {}", length_abs);

    if (!feature_fields.empty() && features_input.empty()) {
        m.set_feature_edges(fields.boundaries);
        // As for `features`: the surface envelope does not hold the curves; this keeps them
        // within its eps.
        m.set_feature_envelope(envelope_size);
        logger().info(
            "{} feature edges: the boundaries of {}",
            m.feature_edges().size(),
            nlohmann::json(feature_fields).dump());
    }
    if (!features_input.empty()) {
        const size_t listed = features.edges.size();
        // The field boundaries join them, without an id.
        for (const auto& e : fields.boundaries) {
            features.edges.push_back(e);
            features.edge_ids.push_back(-1);
        }
        m.set_feature_edges(features.edges, features.edge_ids);
        m.set_fixed_vertices(features.vertices);
        // The surface envelope does not hold the curves; this keeps them within its eps.
        m.set_feature_envelope(envelope_size);
        logger().info(
            "{} feature edges, {} of them from {}{}; {} fixed vertices",
            m.feature_edges().size(),
            listed,
            features_source,
            feature_fields.empty()
                ? ""
                : ", with the boundaries of " + nlohmann::json(feature_fields).dump(),
            features.vertices.size());
    }
    // Whatever the output: the repairs read them where there is a fold (see
    // IsotropicRemeshing::cleanup_triangles), and the mesh may not depend on whether they are
    // written.
    if (!fields.face_fields.empty()) m.set_face_fields(fields.face_values);

    std::shared_ptr<history::Recorder> history;
    if (!history_output.empty()) {
        history = m.start_history([&m](size_t v) { return m.vertex_attrs.at(v).pos; });
        history->history().metadata = component_io.history_metadata(json_params);
        for (const auto& e : m.feature_edges()) {
            history->history().features.push_back({{e[0], e[1]}});
        }
    }

    std::function<void()> write_output;
    if (vtu_output) {
        write_output = [&]() { write_vtu(m, output, fields); };
    }
    const double runtime = run_remeshing(input_path, length_abs, output, m, itrs, write_output);
    if (check_inversions) {
        logger().info("refused {} operations that would fold the surface", m.refused_inversions());
    }

    if (history) {
        m.stop_history();
        // The file as read, the weld to the mesh remeshed, then the remeshing: the history maps
        // the input file to the output file.
        component_io.save_history(
            history::from_input(read, verts, history->history()),
            history_output);
    }

    if (!m.check_mesh_connectivity_validity()) {
        log_and_throw_error("Mesh connectivity is invalid after remeshing!");
    }
    size_t out_nm_e = 0;
    size_t out_nm_v = 0;
    {
        for (const Tuple& e : m.get_edges()) {
            if (m.edge_valence(e) > 2) {
                ++out_nm_e;
            }
        }
        for (const Tuple& t : m.get_vertices()) {
            if (!m.is_manifold_vertex(t.vid(m))) {
                ++out_nm_v;
            }
        }
        logger().info("Output non-manifold: {} edges, {} vertices.", out_nm_e, out_nm_v);
    }

    const std::string report_file = json_params["report"];
    if (component_io.memory() || !report_file.empty()) {
        nlohmann::json report;
        std::vector<double> properties = m.average_len_valen();
        report["avg_length"] = properties[0];
        report["max_length"] = properties[1];
        report["min_length"] = properties[2];
        report["avg_valence"] = properties[3];
        report["max_valence"] = properties[4];
        report["min_valence"] = properties[5];
        report["target_length"] = length_abs;
        report["time_sec"] = runtime;
        report["#v"] = m.get_vertices().size();
        report["#f"] = m.get_faces().size();
        report["nonmanifold_edges"] = out_nm_e;
        report["nonmanifold_vertices"] = out_nm_v;
        report["use_link_condition"] = use_link_condition;
        report["check_inversions"] = check_inversions;
        report["swap_quality"] = swap_quality;
        report["limit_collapse_length"] = limit_collapse_length;
        report["cleanup_angle"] = cleanup_angle;
        report["refused_inversions"] = m.refused_inversions();
        component_io.result.second = report;
        if (!component_io.memory()) {
            std::ofstream fout(report_file);
            fout << std::setw(4) << report;
        }
    }
    if (component_io.memory()) {
        if (features_input.empty() && feature_fields.empty()) {
            component_io.result.first["mesh"] =
                io::triangle_data(m, [&](size_t v) { return m.vertex_attrs[v].pos; });
        } else {
            // What the file route's .vtu output holds: the mesh with vertex_id and the per-face
            // fields, and its feature edges on the same vertex rows, with edge_id.
            MeshMap raw;
            write_vtu(m, "mesh.vtu", fields, &raw);
            component_io.result.first = io::named_outputs(std::move(raw));
        }
    }
}

void isotropic_remeshing(nlohmann::json json_params)
{
    io::ComponentIO context;
    run(std::move(json_params), context);
}
ComponentResult isotropic_remeshing(nlohmann::json json_params, const MeshMap& meshes)
{
    io::ComponentIO context(&meshes);
    run(std::move(json_params), context);
    return context.take_result();
}

} // namespace wmtk::components::isotropic_remeshing