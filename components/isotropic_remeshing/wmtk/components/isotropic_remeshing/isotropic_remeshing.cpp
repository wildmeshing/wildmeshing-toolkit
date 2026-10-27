#include "isotropic_remeshing.hpp"
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/utils/Preallocation.hpp>

#include <igl/Timer.h>
#include <jse/jse.h>

#include <wmtk/utils/getRSS.h>
#include <wmtk/Types.hpp>
#include <wmtk/history/InputHistory.hpp>
#include <wmtk/io/mesh_io.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>
#include <wmtk/utils/io.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <filesystem>
#include <fstream>
#include <map>
#include <set>

#include <isotropic_remeshing_spec.hpp>
#include "IsotropicRemeshing.h"

namespace wmtk::components::isotropic_remeshing {

namespace {

/// The edges between two faces that differ in any of `fields`, triangle fields of the mesh file
/// `path` as io::read_mesh reads it, whose triangles are `tris` in row order. A per-corner field
/// (io::read_mesh's uv and normal: three values per corner, corner after corner) differs across
/// an edge where the two faces disagree on it at either end: a texture seam, say.
std::vector<std::array<size_t, 2>> field_boundaries(
    const std::string& path,
    const std::vector<std::array<size_t, 3>>& tris,
    const std::vector<std::string>& fields)
{
    MeshData mesh;
    try {
        mesh = io::read_mesh(path);
    } catch (const std::exception& e) {
        log_and_throw_error("feature_fields: could not read {}: {}", path, e.what());
    }
    const auto it = mesh.cells.find(CellType::Triangle);
    const Eigen::Index n_faces = it == mesh.cells.end() ? 0 : it->second.connectivity.rows();
    if (size_t(n_faces) != tris.size()) {
        log_and_throw_error(
            "feature_fields: reading {} dropped faces ({} of {} left), so its fields no longer "
            "line up with the triangles",
            path,
            tris.size(),
            n_faces);
    }
    std::vector<std::vector<double>> value(tris.size());
    std::vector<std::array<std::vector<double>, 3>> corner_value(tris.size());
    for (const std::string& name : fields) {
        const auto a = it->second.attributes.find(name);
        if (a == it->second.attributes.end()) {
            log_and_throw_error("feature_fields: {} has no field {}", path, name);
        }
        const DataMatrix<double> values = a->second.doubles();
        const bool corner = io::ComponentIO::reader_corner_attributes.count(name) > 0;
        if (corner && values.cols() % 3 != 0) {
            log_and_throw_error(
                "feature_fields: {} in {} holds a value per corner, but has {} columns",
                name,
                path,
                values.cols());
        }
        const Eigen::Index k = values.cols() / 3;
        for (size_t f = 0; f < tris.size(); ++f) {
            const double* row = values.row(f).data();
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
    std::vector<std::array<size_t, 2>> edges;
    for (const auto& [e, fs] : edge_faces) {
        if (fs.size() != 2) continue;
        const size_t f = fs[0], g = fs[1];
        if (value[f] != value[g] || at(f, e[0]) != at(g, e[0]) || at(f, e[1]) != at(g, e[1])) {
            edges.push_back(e);
        }
    }
    return edges;
}

/// The contents of a "features" file (see the spec), checked against the input mesh.
struct FeatureFile
{
    std::vector<std::array<size_t, 2>> edges;
    std::vector<int64_t> edge_ids;
    std::vector<std::pair<size_t, int64_t>> vertices;
};

FeatureFile read_features(
    const std::string& path,
    const size_t n_vertices,
    const std::vector<std::array<size_t, 3>>& tris)
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
    std::set<std::array<size_t, 2>> mesh_edges;
    for (const auto& t : tris) {
        for (int k = 0; k < 3; ++k) {
            mesh_edges.insert({{std::min(t[k], t[(k + 1) % 3]), std::max(t[k], t[(k + 1) % 3])}});
        }
    }

    FeatureFile out;
    const nlohmann::json edges = j.value("edges", nlohmann::json::array());
    const nlohmann::json ids = j.value("edge_ids", nlohmann::json::array());
    if (!ids.empty() && ids.size() != edges.size()) {
        log_and_throw_error(
            "features: {} has {} edge_ids for {} edges",
            path,
            ids.size(),
            edges.size());
    }
    std::map<std::array<size_t, 2>, int64_t> seen;
    for (size_t i = 0; i < edges.size(); ++i) {
        if (!edges[i].is_array() || edges[i].size() != 2) {
            log_and_throw_error("features: edge {} in {} is not a pair", edges[i].dump(), path);
        }
        const size_t a = vertex(edges[i][0], "edge end"), b = vertex(edges[i][1], "edge end");
        if (a == b) continue; // a B-rep edge of no length: its one vertex, no mesh edge
        const std::array<size_t, 2> e{{std::min(a, b), std::max(a, b)}};
        if (!mesh_edges.count(e)) {
            log_and_throw_error("features: ({}, {}) in {} is not an edge of the input", a, b, path);
        }
        const int64_t id = ids.empty() ? -1 : ids[i].get<int64_t>();
        const auto [it, inserted] = seen.emplace(e, id);
        if (!inserted) {
            if (it->second != id) {
                log_and_throw_error(
                    "features: edge ({}, {}) is listed with ids {} and {} in {}",
                    a,
                    b,
                    it->second,
                    id,
                    path);
            }
            continue;
        }
        out.edges.push_back(e);
        out.edge_ids.push_back(id);
    }
    std::map<size_t, int64_t> fixed;
    for (const nlohmann::json& v : j.value("vertices", nlohmann::json::array())) {
        if (!v.is_array() || v.size() != 2 || !v[1].is_number_integer()) {
            log_and_throw_error("features: vertex {} in {} is not [vertex, id]", v.dump(), path);
        }
        const size_t x = vertex(v[0], "vertex");
        const auto [it, inserted] = fixed.emplace(x, v[1].get<int64_t>());
        if (!inserted && it->second != v[1].get<int64_t>()) {
            log_and_throw_error(
                "features: vertex {} is listed with ids {} and {} in {}",
                x,
                it->second,
                v[1].get<int64_t>(),
                path);
        }
        if (inserted) out.vertices.emplace_back(x, it->second);
    }
    return out;
}

} // namespace

double
run_remeshing(std::string input, double len, std::string output, IsotropicRemeshing& m, int itrs)
{
    igl::Timer timer;
    wmtk::logger().info("target len: {}", len);
    timer.start();
    m.uniform_remeshing(len, itrs);
    timer.stop();

    m.consolidate_mesh();
    if (!output.empty()) m.write_triangle_mesh(output);
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
    const double remove_duplicate_eps = json_params["remove_duplicate_eps"];
    const std::string history_output = json_params["history_output"];
    const std::vector<std::string> feature_fields = json_params["feature_fields"];
    const std::string features_file = json_params["features"];
    // The fields are read from the .msh file the input names; a memory input carries none.
    if (component_io.memory() && !feature_fields.empty()) {
        throw std::runtime_error(
            "isotropic_remeshing: feature_fields reads per-face fields from the input .msh file, "
            "which the memory route does not have");
    }
    double length_abs = json_params["length_abs"];

    wmtk::logger().info("remeshing on {}", input_path);
    std::vector<Eigen::Vector3d> verts;
    std::vector<std::array<size_t, 3>> tris;
    std::pair<Eigen::Vector3d, Eigen::Vector3d> box_minmax;
    std::vector<size_t> modified_nonmanifold_v;
    // With a history: the file as read and the vertex of the mesh each of its vertices became,
    // so that the history starts at the file itself.
    wmtk::io::ReadProvenance read;
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
            history_output.empty() && features_file.empty() ? nullptr : &read);
        if (!features_file.empty()) {
            // The features name vertices of the file; the reader must not renumber them.
            bool same = read.vertex_map.size() == size_t(V.rows());
            for (size_t i = 0; same && i < read.vertex_map.size(); ++i) {
                same = read.vertex_map[i] == int(i);
            }
            if (!same) {
                log_and_throw_error(
                    "features: reading {} renumbered its vertices ({} in the file, {} after "
                    "merging duplicates and dropping unreferenced ones), so {} no longer names "
                    "them. Merging sorts the vertices even where none coincide: set "
                    "remove_duplicate_eps to -1.",
                    input_path,
                    read.vertex_map.size(),
                    V.rows(),
                    features_file);
            }
        }
        box_minmax.first = V.colwise().minCoeff();
        box_minmax.second = V.colwise().maxCoeff();
        VF_to_vectors(V, F, verts, tris);
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

    std::vector<std::array<size_t, 2>> boundaries;
    if (!feature_fields.empty()) {
        boundaries = field_boundaries(input_path, tris, feature_fields);
        if (features_file.empty()) {
            m.set_feature_edges(boundaries);
            logger().info(
                "{} feature edges: the boundaries of {}",
                m.feature_edges().size(),
                nlohmann::json(feature_fields).dump());
        }
    }
    if (!features_file.empty()) {
        const std::string path = resolve_path(root, features_file).string();
        FeatureFile features = read_features(path, verts.size(), tris);
        const size_t listed = features.edges.size();
        // The field boundaries join them, without an id.
        for (const auto& e : boundaries) {
            features.edges.push_back(e);
            features.edge_ids.push_back(-1);
        }
        m.set_feature_edges(features.edges, features.edge_ids);
        m.set_fixed_vertices(features.vertices);
        logger().info(
            "{} feature edges, {} of them from {}{}; {} fixed vertices",
            m.feature_edges().size(),
            listed,
            path,
            feature_fields.empty()
                ? ""
                : ", with the boundaries of " + nlohmann::json(feature_fields).dump(),
            features.vertices.size());
    }

    std::shared_ptr<history::Recorder> history;
    if (!history_output.empty()) {
        history = m.start_history([&m](size_t v) { return m.vertex_attrs.at(v).pos; });
        history->history().metadata = component_io.history_metadata(json_params);
        for (const auto& e : m.feature_edges()) {
            history->history().features.push_back({{e[0], e[1]}});
        }
    }

    const double runtime = run_remeshing(input_path, length_abs, output, m, itrs);
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
        report["refused_inversions"] = m.refused_inversions();
        component_io.result.second = report;
        if (!component_io.memory()) {
            std::ofstream fout(report_file);
            fout << std::setw(4) << report;
        }
    }
    if (component_io.memory())
        component_io.result.first["mesh"] =
            io::triangle_data(m, [&](size_t v) { return m.vertex_attrs[v].pos; });
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