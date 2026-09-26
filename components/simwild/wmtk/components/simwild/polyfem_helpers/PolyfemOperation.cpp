#include "PolyfemOperation.hpp"

#include "MeshReduction.hpp"

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/DriverPrologue.hpp>
#include <wmtk/utils/io.hpp>

#include <algorithm>
#include <map>
#include <sstream>

namespace wmtk::components::simwild::polyfem_helpers {

std::string operation_input_path(const nlohmann::json& json_params)
{
    const std::vector<std::string> inputs =
        wmtk::utils::resolve_input_paths(
            json_params,
            json_params["input_dir"].get<std::string>());
    if (inputs.size() != 1) {
        log_and_throw_error(
            "operation {} solves on one already-tagged .msh, but {} input files were given",
            json_params["operation"].get<std::string>(),
            inputs.size());
    }
    return inputs.front();
}

std::filesystem::path sim_dir(const std::string& output, const std::string& subdir)
{
    std::filesystem::path out_dir = std::filesystem::path(output).parent_path();
    if (out_dir.empty()) {
        out_dir = ".";
    }
    std::filesystem::create_directories(out_dir);
    const std::filesystem::path sim = std::filesystem::canonical(out_dir) / subdir;
    std::filesystem::create_directories(sim);
    return std::filesystem::canonical(sim);
}

ReducedMesh reduce_mesh(
    const std::string& input,
    const std::vector<std::string>& ambient_like_tags,
    const std::filesystem::path& sim_in_dir,
    const bool inputs_only)
{
    ReducedMesh out;
    out.path = sim_in_dir / (std::filesystem::path(input).stem().string() + "_polyfem.msh");
    logger().info("[reduce mesh for polyfem]");
    out.content = polyfem_reduced_msh(input, ambient_like_tags);
    if (inputs_only) {
        write_polyfem_reduced_msh(out.path.string(), out.content);
    }
    out.info = get_mesh_info(out.content);
    logger().info("Reduced material tags : {}  dim={}", out.info.tags, out.info.dim);
    return out;
}

void emit_interface_constraint(
    const InterfaceConstraint& generated,
    const std::filesystem::path& dir,
    const bool with_collision_proxy,
    const bool inputs_only,
    SolveInputs& memory)
{
    const auto path = [&dir](const char* name) { return (dir / name).string(); };
    logger().info("Writing:");
    // The OBJ is written in every mode, although a normal run hands polyfem the proxy from
    // `memory` and never reads this file: it is the only convenient way to see which faces the
    // selection picked, and the Python engine always wrote it.
    write_collision_mesh_obj(path("interface_collision.obj"), generated.collision_mesh);
    if (inputs_only) {
        write_constraint_hdf5(path("interface_constraint.hdf5"), generated.fitting);
        write_constraint_hdf5(path("interface_constraint_laplacian.hdf5"), generated.laplacian);
        if (with_collision_proxy) {
            write_linear_map_hdf5(path("interface_linear_map.hdf5"), generated.linear_map);
            write_collision_body_ids_txt(
                path("collision_body_ids.txt"),
                generated.collision_body_ids);
        }
        return;
    }
    memory.constraints.emplace(path("interface_constraint.hdf5"), generated.fitting);
    memory.constraints.emplace(path("interface_constraint_laplacian.hdf5"), generated.laplacian);
    if (with_collision_proxy) {
        memory.collision_meshes.emplace(path("interface_collision.obj"), generated.collision_mesh);
        memory.linear_maps.emplace(path("interface_linear_map.hdf5"), generated.linear_map);
        memory.collision_body_ids.emplace(
            path("collision_body_ids.txt"),
            generated.collision_body_ids);
    }
}

bool allow_out_of_iterations(const OrderedJson& doc)
{
    const auto solver = doc.find("solver");
    if (solver == doc.end() || !solver->is_object()) return false;
    const auto nonlinear = solver->find("nonlinear");
    if (nonlinear == solver->end() || !nonlinear->is_object()) return false;
    const auto flag = nonlinear->find("allow_out_of_iterations");
    return flag != nonlinear->end() && flag->get<bool>();
}

std::unique_ptr<PolyfemBackend> operation_backend(PreparedOperation& prepared)
{
    return in_process_backend(std::move(prepared.inputs));
}

namespace {

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

void check_result_layout(const std::string& input)
{
    wmtk::MshData msh;
    msh.load(input);
    require_rebuildable(msh, input);
}

void write_operation_result(const PreparedOperation& prepared, const Eigen::MatrixXd& solution)
{
    logger().info("[write deformed msh]");
    wmtk::MshData input;
    input.load(prepared.input);
    require_rebuildable(input, prepared.input);
    const int mesh_dim = input.get_physical_groups().front().dim;
    const size_t n_nodes = input.m_spec.nodes.num_nodes;
    if (size_t(solution.rows()) != n_nodes || solution.cols() != mesh_dim) {
        log_and_throw_error(
            "the solution is {} x {}, but {} has {} nodes in dimension {}",
            solution.rows(),
            solution.cols(),
            prepared.input,
            n_nodes,
            mesh_dim);
    }
    // The mesh nodes have the tags 1..n, so they are rows 0..n-1 of the solution
    // (`node_tag_to_index`). `u_mesh = u / scale`, THEN the add in `rebuild`: two roundings, in
    // the Python's order.
    const MatrixXd u_mesh = solution / prepared.cfg["scale"].get<double>();
    wmtk::MshData output = rebuild(input, &u_mesh);
    output.save(prepared.output + ".msh", /*binary=*/true);
    logger().info("  Out : {}.msh", prepared.output);
}

} // namespace wmtk::components::simwild::polyfem_helpers
