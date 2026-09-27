#include "polyfem_operations.hpp"

#include "polyfem_helpers/LaplacianSmoothing.hpp"
#include "polyfem_helpers/MinimumSeparation.hpp"
#include "polyfem_helpers/PythonFormat.hpp"

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/io.hpp>
#include <wmtk/utils/resolve_path.hpp>

// h5pp comes in through polyfem::polyfem (polyfem -> paraviewo -> h5pp, with HDF5 on), the same
// way polyfem itself reaches it (`#include <h5pp/h5pp.h>` in its sources), so the component's
// CMakeLists needs no extra link target for it.
#include <h5pp/h5pp.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

namespace wmtk::components::simwild {

namespace fs = std::filesystem;
using polyfem_helpers::CollisionObj;
using polyfem_helpers::ConstraintHdf5;
using polyfem_helpers::GeneratedInputs;
using polyfem_helpers::LinearMapHdf5;
using polyfem_helpers::OperationFailed;
using polyfem_helpers::OperationResult;
using polyfem_helpers::OrderedJson;
using polyfem_helpers::python_repr;
using polyfem_helpers::ReducedMsh;
using polyfem_helpers::SolveReport;
using polyfem_helpers::TaggedMesh;

namespace {

// ---------------------------------------------------------------------------
// The files inputs_only writes
// ---------------------------------------------------------------------------

/// Write the simulation JSON the way `json.dumps(doc, indent=4)` + `Path.write_text` does: four
/// spaces of indent and NO trailing newline.
void write_polyfem_json(const std::filesystem::path& path, const OrderedJson& doc)
{
    std::ofstream out(path);
    if (!out.is_open()) {
        log_and_throw_error("Unable to open {} for writing", path.string());
    }
    out << doc.dump(4);
}

/// Write a constraint file, dataset for dataset what the Python's three constraint writers write.
void write_constraint_hdf5(const std::string& path, const ConstraintHdf5& constraint)
{
    h5pp::File file(path, h5pp::FileAccess::REPLACE);
    file.writeDataset(constraint.local2global, "local2global");
    file.writeDataset(constraint.a.rows, "A_triplets/rows");
    file.writeDataset(constraint.a.cols, "A_triplets/cols");
    file.writeDataset(constraint.a.values, "A_triplets/values");
    file.writeDataset(
        std::vector<int64_t>{constraint.shape[0], constraint.shape[1]},
        "A_triplets/shape");
    file.writeDataset(constraint.b, "b", {constraint.b_rows, constraint.b_cols});
}

/// Write the linear map file, dataset and attribute for what the Python writes.
void write_linear_map_hdf5(const std::string& path, const LinearMapHdf5& map)
{
    h5pp::File file(path, h5pp::FileAccess::REPLACE);
    file.writeDataset(map.rows, "weight_triplets/rows");
    file.writeDataset(map.cols, "weight_triplets/cols");
    file.writeDataset(map.values, "weight_triplets/values");
    // An ATTRIBUTE on the group, not a dataset -- polyfem CollisionProxy.cpp reads it there.
    file.writeAttribute(
        std::vector<int64_t>{map.shape[0], map.shape[1]},
        "weight_triplets",
        "shape");

    logger().info("  linear map : {}  (shape [{}, {}])", path, map.shape[0], map.shape[1]);
}

/// Write the OBJ, byte-identical to the Python writer, floats included -- see PythonFormat.hpp.
void write_collision_mesh_obj(const std::string& path, const CollisionObj& obj)
{
    std::ofstream f(path, std::ios::binary); // binary: no CRLF translation, the bytes must match
    if (!f) {
        log_and_throw_error("Cannot open {} for writing", path);
    }
    f << "# Interface collision mesh\n";
    for (const auto& [x, y, z] : obj.vertices) {
        f << "v " << python_repr(x) << " " << python_repr(y) << " " << python_repr(z) << "\n";
    }
    // OBJ is 1-based
    for (const auto& t : obj.faces) {
        f << "f " << t[0] + 1 << " " << t[1] + 1 << " " << t[2] + 1 << "\n";
    }
    for (const auto& e : obj.edges) {
        f << "l " << e[0] + 1 << " " << e[1] + 1 << "\n";
    }
    if (!obj.faces.empty()) {
        logger().info(
            "  collision  : {}  ({} verts, {} faces)",
            path,
            obj.vertices.size(),
            obj.faces.size());
    } else {
        logger().info(
            "  collision  : {}  ({} verts, {} edges)",
            path,
            obj.vertices.size(),
            obj.edges.size());
    }
}

/// Mirrors `constraints.write_collision_body_ids_txt`: one space-separated line of collision body
/// ids per face/edge, row order matching the OBJ.
void write_collision_body_ids_txt(
    const std::string& path,
    const std::vector<std::vector<int64_t>>& face_tags)
{
    std::ofstream f(path, std::ios::binary);
    if (!f) {
        log_and_throw_error("Cannot open {} for writing", path);
    }
    for (const auto& tags : face_tags) {
        for (size_t i = 0; i < tags.size(); ++i) {
            if (i != 0) f << " ";
            f << tags[i];
        }
        f << "\n";
    }
    logger().info("  body IDs   : {}  ({} faces)", path, face_tags.size());
}

/**
 * @brief Save the reduced mesh to `output_msh` with wmtk::MshData, as binary msh 4.1: "ambient"
 * and "body", each a physical group with one entity of its own tag; every node in the ambient
 * entity's node block, and an empty node block for the body entity.
 *
 * Binary, because MshData's ASCII writer (mshio's) prints coordinates through a default ostream,
 * which keeps 6 significant digits and would round every coordinate away. Binary stores the doubles
 * themselves, so the file carries exactly the coordinates the input had, which is at least as
 * faithful as the gmsh ASCII (%.16g) the Python engine writes -- and it is what makes the file and
 * the arrays the same mesh.
 */
void write_polyfem_reduced_msh(const std::string& output_msh, const ReducedMsh& reduced)
{
    const int64_t n_body = int64_t(reduced.cells.rows()) - reduced.n_ambient;
    const auto vertex = [&reduced](const size_t i) {
        return reduced.vertices.row(Eigen::Index(i));
    };
    const auto ambient_cell = [&reduced](const size_t i) {
        return reduced.cells.row(Eigen::Index(i));
    };
    const auto body_cell = [&reduced](const size_t i) {
        return reduced.cells.row(Eigen::Index(reduced.n_ambient) + Eigen::Index(i));
    };

    // Every node on the ambient entity. The body entity gets an empty node block, which MshData
    // takes as "the element vertex ids are global", so both groups index the one node block.
    wmtk::MshData msh;
    if (reduced.dim == 3) {
        msh.add_tet_vertices(size_t(reduced.vertices.rows()), vertex);
        msh.add_tets(size_t(reduced.n_ambient), ambient_cell);
        msh.add_physical_group("ambient");
        msh.add_tet_vertices();
        msh.add_tets(size_t(n_body), body_cell);
        msh.add_physical_group("body");
    } else {
        msh.add_face_vertices(size_t(reduced.vertices.rows()), vertex);
        msh.add_faces(size_t(reduced.n_ambient), ambient_cell);
        msh.add_physical_group("ambient");
        msh.add_face_vertices();
        msh.add_faces(size_t(n_body), body_cell);
        msh.add_physical_group("body");
    }
    msh.save(output_msh, /*binary=*/true);
    logger().info(
        "  reduced  : {}  ({} ambient + {} body {})",
        output_msh,
        reduced.n_ambient,
        n_body,
        reduced.dim == 3 ? "tets" : "triangles");
}

/// inputs_only: the simulation JSON and every other generated file, each written to the path it
/// is named by, which is in the simulation input directory.
void write_generated_inputs(const GeneratedInputs& inputs)
{
    fs::create_directories(inputs.sim_json_path.parent_path());
    logger().info("Writing:");
    write_polyfem_json(inputs.sim_json_path, inputs.sim_json);
    for (const auto& [path, content] : inputs.files.meshes) {
        write_polyfem_reduced_msh(path, content);
    }
    for (const auto& [path, content] : inputs.files.constraints) {
        write_constraint_hdf5(path, content);
    }
    for (const auto& [path, content] : inputs.files.collision_meshes) {
        write_collision_mesh_obj(path, content);
    }
    for (const auto& [path, content] : inputs.files.linear_maps) {
        write_linear_map_hdf5(path, content);
    }
    for (const auto& [path, content] : inputs.files.collision_body_ids) {
        write_collision_body_ids_txt(path, content);
    }
}

// ---------------------------------------------------------------------------
// A run
// ---------------------------------------------------------------------------

/**
 * @brief `json_params` as they are handed to polyfem_helpers: `input` the one input file resolved
 * against `input_dir`, as every other simwild operation resolves it, and `output` with its
 * directory created and resolved.
 *
 * polyfem_helpers names every generated file after these two (`generated_dir`) and puts the names
 * verbatim into the simulation JSON, where the Python engine put resolved ones. Resolved, because
 * on this machine /tmp and /var are symlinks, so an unresolved path would name the same directory
 * by a different name than the Python did.
 */
nlohmann::json resolved_params(nlohmann::json params)
{
    params["input"] = nlohmann::json::array({wmtk::utils::resolve_path(
                                                 params["input_dir"].get<std::string>(),
                                                 polyfem_helpers::operation_input(params))
                                                 .string()});
    const fs::path output = params["output"].get<std::string>();
    fs::path out_dir = output.parent_path();
    if (out_dir.empty()) {
        out_dir = ".";
    }
    fs::create_directories(out_dir);
    params["output"] = (fs::canonical(out_dir) / output.filename()).string();
    return params;
}

/// The input mesh of an operation, read from its file.
TaggedMesh read_input(const std::string& input, const std::string& output)
{
    logger().info("Input  : {}", input);
    logger().info("Output : {}.msh", output);
    logger().info("Reading {} ...", input);
    return TaggedMesh(input);
}

/**
 * @brief Each solve's polyfem log, written into `dir` (created) under the name the Python engine
 * gave the file: `polyfem_iter_<i>.log` for iteration i of the separation loop, `polyfem_probe.log`
 * for its probe, and `polyfem.log` for smoothing's one solve. Each file holds the solve's output as
 * polyfem printed it (`SolveReport::log`), which polyfem has already printed to the console too.
 *
 * Separation first deletes every `polyfem_*.log` already there, as the Python did
 * (`for stale in sim_out_dir.glob("polyfem_*.log"): stale.unlink()`): a shorter rerun must not
 * leave the iteration logs of a longer one behind, which would be read as this run's. Smoothing
 * has the one file, which it overwrites.
 */
void write_solve_logs(const fs::path& dir, const std::vector<SolveReport>& solves, bool separation)
{
    fs::create_directories(dir);
    if (separation) {
        for (const auto& entry : fs::directory_iterator(dir)) {
            const std::string name = entry.path().filename().string();
            if (name.rfind("polyfem_", 0) == 0 && name.size() > 12 &&
                name.compare(name.size() - 4, 4, ".log") == 0) {
                fs::remove(entry.path());
            }
        }
    }
    for (const SolveReport& report : solves) {
        std::string name = "polyfem.log";
        if (separation) {
            name = report.iteration.has_value()
                       ? fmt::format("polyfem_iter_{}.log", *report.iteration)
                       : "polyfem_probe.log";
        }
        std::ofstream file(dir / name, std::ios::binary);
        if (!file) {
            log_and_throw_error("Cannot open {} for writing", (dir / name).string());
        }
        for (const std::string& line : report.log) {
            file << line;
        }
    }
}

/**
 * @brief With `write_simulation_json`, the simulation JSON polyfem was handed on the last solve
 * that ran -- failed or rolled back included, the document the Python engine left on disk -- written
 * to the path inputs_only writes the generated one to. Without it, nothing.
 */
void write_last_simulation_json(
    const nlohmann::json& params,
    const std::vector<SolveReport>& solves,
    const bool separation)
{
    if (!params["write_simulation_json"].get<bool>() || solves.empty()) {
        return;
    }
    const fs::path path =
        separation ? polyfem_helpers::generated_dir(params, "sep_input") / "separation.json"
                   : polyfem_helpers::generated_dir(params, "smooth_input") / "smoothing.json";
    fs::create_directories(path.parent_path());
    write_polyfem_json(path, solves.back().sim_json);
    logger().info("  simulation JSON of the last solve : {}", path.string());
}

/// One operation on files: `generate` and `solve` are the operation's pair in polyfem_helpers, and
/// `separation` tells minimum_separation (sep_output, a log per solve of its loop) from
/// laplacian_smoothing (smooth_output, the one polyfem.log).
void run_operation(
    const nlohmann::json& json_params,
    GeneratedInputs (*generate)(const TaggedMesh&, const nlohmann::json&),
    OperationResult (*solve)(const TaggedMesh&, const nlohmann::json&),
    const bool separation)
{
    const nlohmann::json params = resolved_params(json_params);
    const std::string input = polyfem_helpers::operation_input(params);
    const std::string output = json_params["output"].get<std::string>();
    const TaggedMesh mesh = read_input(input, output);

    if (params["inputs_only"].get<bool>()) {
        write_generated_inputs(generate(mesh, params));
        return;
    }

    check_result_layout(input);
    const fs::path sim_out_dir =
        polyfem_helpers::generated_dir(params, separation ? "sep_output" : "smooth_output");
    OperationResult result;
    try {
        result = solve(mesh, params);
    } catch (const OperationFailed& e) {
        // The failed solve's log is what says why it failed.
        write_solve_logs(sim_out_dir, e.solves, separation);
        write_last_simulation_json(params, e.solves, separation);
        throw;
    }
    write_solve_logs(sim_out_dir, result.solves, separation);
    write_last_simulation_json(params, result.solves, separation);
    write_operation_result(input, output, result.displacement);
}

// ---------------------------------------------------------------------------
// The result
// ---------------------------------------------------------------------------

/// gmsh's element type for the linear simplex of each dimension -- 2-node line, 3-node triangle,
/// 4-node tetrahedron -- the only element types wmtk::MshData writes and reads.
int linear_simplex_type(const int dim)
{
    return dim == 1 ? 1 : dim == 2 ? 2 : 4;
}

/// The refusal of an input `write_operation_result` cannot write its result into; `what` says why.
void refuse_layout(const std::string& path, const std::string& what)
{
    log_and_throw_error(
        "{} cannot take the solution: the result is this file rebuilt with wmtk::MshData, which "
        "reproduces only the layout MshData writes -- one entity per physical group, with the "
        "group's tag; every mesh node on the first group's entity; node and element tags 1, 2, ... "
        "in group order -- and {}",
        path,
        what);
}

/**
 * @brief Throw unless MshData::get_VF can read every block of `msh` without leaving its arrays.
 *
 * get_VF puts a node at row (its tag - the first tag of its block) and an element at row (its tag
 * - the first tag of its block), strides through an element block by the size of the linear
 * simplex of the block's dimension, and reads the first tag of the first node block of that
 * dimension. So: consecutive tags within every block, linear simplices only, and a non-empty node
 * block of every dimension that has elements.
 */
void require_readable_by_get_vf(const wmtk::MshData& msh, const std::string& path)
{
    std::map<int, bool> first_node_block_filled; // dimension -> its first node block has nodes
    for (const auto& block : msh.m_spec.nodes.entity_blocks) {
        first_node_block_filled.emplace(block.entity_dim, block.num_nodes_in_block > 0);
        for (size_t i = 0; i < block.tags.size(); ++i) {
            if (block.tags[i] != block.tags.front() + i) {
                refuse_layout(path, "the node tags of a block are not consecutive");
            }
        }
    }
    for (const auto& block : msh.m_spec.elements.entity_blocks) {
        if (block.element_type != linear_simplex_type(block.entity_dim)) {
            refuse_layout(
                path,
                fmt::format(
                    "it holds elements of gmsh type {}, not linear simplices",
                    block.element_type));
        }
        if (block.num_elements_in_block == 0) {
            continue;
        }
        const auto filled = first_node_block_filled.find(block.entity_dim);
        if (filled == first_node_block_filled.end() || !filled->second) {
            refuse_layout(
                path,
                fmt::format(
                    "it has elements of dimension {} but no nodes in its first node block of that "
                    "dimension",
                    block.entity_dim));
        }
        const size_t stride = size_t(block.entity_dim) + 2;
        for (size_t j = 0; j < block.num_elements_in_block; ++j) {
            if (block.data[j * stride] != block.data.front() + j) {
                refuse_layout(path, "the element tags of a block are not consecutive");
            }
        }
    }
}

/// A vertex block of dimension `dim` holding the rows of `V` (x, y, z).
void add_vertices(wmtk::MshData& msh, const int dim, const MatrixXd& V)
{
    const auto row = [&V](const size_t i) { return V.row(Eigen::Index(i)); };
    switch (dim) {
    case 1: msh.add_edge_vertices(size_t(V.rows()), row); break;
    case 2: msh.add_face_vertices(size_t(V.rows()), row); break;
    default: msh.add_tet_vertices(size_t(V.rows()), row); break;
    }
}

/// An element block of dimension `dim` holding the rows of `F`, in the 0-based numbering
/// MshData::get_VF returns them in.
void add_simplices(wmtk::MshData& msh, const int dim, const MatrixXi& F)
{
    const auto row = [&F](const size_t i) { return F.row(Eigen::Index(i)); };
    switch (dim) {
    case 1: msh.add_edges(size_t(F.rows()), row); break;
    case 2: msh.add_faces(size_t(F.rows()), row); break;
    default: msh.add_tets(size_t(F.rows()), row); break;
    }
}

/// `input` written again group by group with MshData's own writer: the first group's nodes moved
/// by `u_mesh` (mesh units, one row per node of that group) unless it is null, every other block
/// copied.
wmtk::MshData rebuild(wmtk::MshData& input, const MatrixXd* u_mesh)
{
    const auto& groups = input.get_physical_groups();
    const int mesh_dim = groups.front().dim;
    wmtk::MshData output;
    for (size_t k = 0; k < groups.size(); ++k) {
        MatrixXd V;
        MatrixXi F;
        input.get_VF(groups[k], V, F);
        if (k == 0) {
            // In 2D the third coordinate stays the file's.
            if (u_mesh != nullptr) {
                V.leftCols(mesh_dim) += u_mesh->topRows(V.rows());
            }
            add_vertices(output, mesh_dim, V);
        } else if (groups[k].dim == mesh_dim) {
            output.add_empty_vertices(mesh_dim);
        } else {
            add_vertices(output, groups[k].dim, V);
        }
        add_simplices(output, groups[k].dim, F);
        output.add_physical_group(groups[k].name);
    }
    return output;
}

/**
 * @brief What MshData writes for `msh` -- binary msh 4.1, whichever format it was read from --
 * without the two things that carry no content: the entities' bounding boxes and the node blocks
 * without nodes.
 *
 * MshData derives each box from the entity's node block and writes a node block, empty or not, for
 * every group; a mesh built with mshio as pysimwild's conftest lays it out (the fixtures of
 * tests/test_polyfem_in_process.cpp) stores zero boxes and no node block for a group without
 * nodes of its own. Compared with both, the rebuild refused every such fixture.
 */
std::string content(wmtk::MshData msh)
{
    auto& spec = msh.m_spec;
    auto& blocks = spec.nodes.entity_blocks;
    blocks.erase(
        std::remove_if(
            blocks.begin(),
            blocks.end(),
            [](const auto& block) { return block.num_nodes_in_block == 0; }),
        blocks.end());
    spec.nodes.num_entity_blocks = blocks.size();
    for (auto& e : spec.entities.points) e.x = e.y = e.z = 0.0;
    const auto clear_box = [](auto& entities) {
        for (auto& e : entities) e.min_x = e.min_y = e.min_z = e.max_x = e.max_y = e.max_z = 0.0;
    };
    clear_box(spec.entities.curves);
    clear_box(spec.entities.surfaces);
    clear_box(spec.entities.volumes);
    std::ostringstream out;
    msh.save(out, /*binary=*/true);
    return out.str();
}

/**
 * @brief Throw unless `write_operation_result` can write its result for `input`: get_VF can read
 * it, and rebuilding it with nothing moved gives the same content (`content`), so the result is
 * this file with only the mesh nodes moved.
 *
 * MshData writes every group it is handed as one entity carrying the group's tag, the mesh nodes
 * on the first group's entity and a lower-dimensional group's own nodes on its entity, with node
 * and element tags 1, 2, ... in group order. Every file it wrote passes (SimWildMesh::write_msh,
 * SimWildMeshTri::write_msh, with or without the envelope group); any other layout, or node or
 * element data, is refused.
 */
void require_rebuildable(wmtk::MshData& input, const std::string& path)
{
    if (input.get_physical_groups().empty()) {
        refuse_layout(path, "it has no physical group");
    }
    require_readable_by_get_vf(input, path);
    if (content(rebuild(input, nullptr)) != content(input)) {
        refuse_layout(path, "rebuilding it with nothing moved does not give the same content");
    }
}

} // namespace

void minimum_separation(const nlohmann::json& json_params)
{
    run_operation(
        json_params,
        polyfem_helpers::minimum_separation_inputs,
        polyfem_helpers::minimum_separation,
        /*separation=*/true);
}

void laplacian_smoothing(const nlohmann::json& json_params)
{
    run_operation(
        json_params,
        polyfem_helpers::laplacian_smoothing_inputs,
        polyfem_helpers::laplacian_smoothing,
        /*separation=*/false);
}

void check_result_layout(const std::string& input)
{
    wmtk::MshData msh;
    msh.load(input);
    require_rebuildable(msh, input);
}

void write_operation_result(
    const std::string& input,
    const std::string& output,
    const Eigen::MatrixXd& displacement)
{
    logger().info("[write deformed msh]");
    wmtk::MshData input_msh;
    input_msh.load(input);
    require_rebuildable(input_msh, input);
    const int mesh_dim = input_msh.get_physical_groups().front().dim;
    const size_t n_nodes = input_msh.m_spec.nodes.num_nodes;
    if (size_t(displacement.rows()) != n_nodes || displacement.cols() != mesh_dim) {
        log_and_throw_error(
            "the displacement is {} x {}, but {} has {} nodes in dimension {}",
            displacement.rows(),
            displacement.cols(),
            input,
            n_nodes,
            mesh_dim);
    }
    // The mesh nodes have the tags 1..n, so they are rows 0..n-1 of the displacement
    // (`node_tag_to_index`).
    wmtk::MshData output_msh = rebuild(input_msh, &displacement);
    output_msh.save(output + ".msh", /*binary=*/true);
    logger().info("  Out : {}.msh", output);
}

} // namespace wmtk::components::simwild
