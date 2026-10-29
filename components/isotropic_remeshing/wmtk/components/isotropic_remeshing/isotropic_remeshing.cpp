#include "isotropic_remeshing.hpp"
#include <wmtk/io/ComponentIO.hpp>
#include <wmtk/utils/Preallocation.hpp>

#include <igl/Timer.h>
#include <jse/jse.h>

#include <wmtk/utils/getRSS.h>
#include <wmtk/Types.hpp>
#include <wmtk/history/InputHistory.hpp>
#include <wmtk/io/read_triangle_mesh.hpp>
#include <wmtk/utils/io.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <map>

#include <isotropic_remeshing_spec.hpp>
#include "IsotropicRemeshing.h"

namespace wmtk::components::isotropic_remeshing {

namespace {

/// The edges between two faces that differ in any of `fields`, per-face or per-corner fields of
/// the gmsh file `path`, whose triangles are `tris` in file order. A per-corner field differs
/// across an edge where the two faces disagree on it at either end: a texture seam, say.
std::vector<std::array<size_t, 2>> field_boundaries(
    const std::string& path,
    const std::vector<std::array<size_t, 3>>& tris,
    const std::vector<std::string>& fields)
{
    MshData msh;
    msh.load(path);
    if (!msh.is_single_surface_block()) {
        log_and_throw_error(
            "feature_fields: {} is not a single-block surface file, as wmtk writes one; its "
            "fields cannot be matched to its triangles",
            path);
    }
    if (msh.get_num_faces() != tris.size()) {
        log_and_throw_error(
            "feature_fields: reading {} dropped faces ({} of {} left), so its fields no longer "
            "line up with the triangles",
            path,
            tris.size(),
            msh.get_num_faces());
    }
    const mshio::ElementBlock* triangles = &msh.m_spec.elements.entity_blocks.front();
    std::vector<std::vector<double>> value(tris.size());
    std::vector<std::array<std::vector<double>, 3>> corner_value(tris.size());
    const auto names = msh.get_face_attribute_names();
    for (const std::string& name : fields) {
        if (std::find(names.begin(), names.end(), name) != names.end()) {
            msh.extract_face_attribute(name, [&value](size_t i, const std::vector<double>& data) {
                if (i < value.size()) value[i].insert(value[i].end(), data.begin(), data.end());
            });
            continue;
        }
        bool found = false;
        for (const mshio::Data& data : msh.m_spec.element_node_data) {
            if (data.header.string_tags.empty() || data.header.string_tags.front() != name)
                continue;
            found = true;
            for (const mshio::DataEntry& entry : data.entries) {
                const size_t f = entry.tag - triangles->data.front();
                const size_t k = entry.data.size() / 3;
                if (f >= tris.size()) continue;
                for (int c = 0; c < 3; ++c) {
                    auto& cv = corner_value[f][c];
                    cv.insert(
                        cv.end(),
                        entry.data.begin() + c * k,
                        entry.data.begin() + (c + 1) * k);
                }
            }
        }
        if (!found) log_and_throw_error("feature_fields: {} has no field {}", path, name);
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
            history_output.empty() ? nullptr : &read);
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

    if (!feature_fields.empty()) {
        m.set_feature_edges(field_boundaries(input_path, tris, feature_fields));
        logger().info(
            "{} feature edges: the boundaries of {}",
            m.feature_edges().size(),
            nlohmann::json(feature_fields).dump());
    }

    std::shared_ptr<history::Recorder> history;
    if (!history_output.empty()) {
        history = m.start_history([&m](size_t v) { return m.vertex_attrs.at(v).pos; });
        history->history().metadata = json_params.dump();
        for (const auto& e : m.feature_edges()) {
            history->history().feature_edges.push_back({{e[0], e[1]}});
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
        const history::SurfaceHistory chain = history::from_input(read, verts, history->history());
        chain.save(history_output);
        logger().info("history: {} edits saved to {}", chain.edits.size(), history_output);
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
    return std::move(context.result);
}

} // namespace wmtk::components::isotropic_remeshing