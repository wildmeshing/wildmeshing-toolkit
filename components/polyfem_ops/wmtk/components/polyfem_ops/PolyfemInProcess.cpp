#include "PolyfemRunner.hpp"

#include <wmtk/utils/Logger.hpp>

#include <polyfem/State.hpp>
#include <polyfem/mesh/GeometryReader.hpp>
#include <polyfem/mesh/Mesh.hpp>
#include <polyfem/solver/forms/SmoothContactForm.hpp>
#include <polyfem/time_integrator/ImplicitTimeIntegrator.hpp>
#include <polyfem/utils/JSONUtils.hpp>
#include <polyfem/utils/Logger.hpp>

#include <ipc/utils/logger.hpp>

#include <spdlog/sinks/base_sink.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <mutex>
#include <optional>
#include <unordered_map>

namespace wmtk::components::polyfem_ops {

namespace {

/**
 * @brief The warm start between two solves, in memory: exactly the three matrices polyfem's
 * `ImplicitTimeIntegrator::save_state` writes into `curr_state.hdf5` and
 * `State::initial_solution` / `initial_velocity` / `initial_acceleration` read back out of
 * `prev_state.hdf5`.
 *
 * The round trip through hdf5 is lossless -- `write_matrix`/`read_matrix` store float64 -- so
 * carrying the matrices instead of the file changes no number; what it removes is a file write and
 * a file read per outer iteration.
 */
struct SolverState
{
    Eigen::MatrixXd solution; ///< "u": the time integrator's x_prevs(), one column per stored step
    Eigen::MatrixXd velocity; ///< "v": v_prevs()
    Eigen::MatrixXd acceleration; ///< "a": a_prevs()
};

/**
 * @brief The active distance polyfem's contact form measures at `sol`, or nothing when polyfem
 * would not have logged one.
 *
 * `polyfem::solver::SmoothContactForm::post_step` is the only place the
 * "Minimum distance during solve: ..., active distance: ..., dhat: ..." line comes from, and it
 * computes exactly these two calls on `collision_set()` -- the set cached at the last solution the
 * solver visited, which is `sol` -- and prints nothing at all when the minimum distance is
 * infinite. Reproducing that guard is what makes "no line in the log" and "no value here" the same
 * event for the loops, which stop on it with "contact not triggered".
 */
std::optional<double> active_distance_from_contact_form(
    const polyfem::State& state,
    const Eigen::MatrixXd& sol);

/**
 * @brief The sink that turns polyfem's own log records into `polyfem_iter_<i>.log` (and into the
 * lines whose last "Finished:" line `check_polyfem_success` quotes when a solve failed).
 *
 * The Python engine copies the executable's stdout into that file. In process there is no child
 * stdout to copy, so the text is taken where it is produced: spdlog's default pattern formats each
 * record exactly as the `stdout_color_sink` the executable installs formats it, and the escapes
 * polyfem embeds in a few messages with `fmt::fg` (the "timing" tag) are stripped here just as the
 * Python strips them out of its capture.
 *
 * `stop()` exists because `State::init` REPLACES polyfem's global logger on every solve: a logger
 * this sink was attached to can outlive the solve if something still holds it, and after stop()
 * such a straggler writes nothing instead of writing into a closed file.
 */
class LogCapture : public spdlog::sinks::base_sink<std::mutex>
{
public:
    explicit LogCapture(const std::filesystem::path& log_path)
        : m_file(log_path)
    {
        if (!m_file.is_open()) {
            log_and_throw_error("Unable to open {} for writing", log_path.string());
        }
    }

    void stop()
    {
        std::lock_guard<std::mutex> lock(base_sink<std::mutex>::mutex_);
        m_stopped = true;
        m_file.close();
    }

    /// The captured records, split on '\n' exactly as the Python splits the executable's output.
    std::vector<std::string> lines()
    {
        std::lock_guard<std::mutex> lock(base_sink<std::mutex>::mutex_);
        return m_lines;
    }

protected:
    void sink_it_(const spdlog::details::log_msg& msg) override
    {
        if (m_stopped) {
            return;
        }
        spdlog::memory_buf_t formatted;
        base_sink<std::mutex>::formatter_->format(msg, formatted);
        const std::string text = strip_ansi(fmt::to_string(formatted));
        m_file << text;
        for (auto& line : split_lines(text)) {
            m_lines.push_back(std::move(line));
        }
    }

    void flush_() override
    {
        if (!m_stopped) {
            m_file.flush();
        }
    }

private:
    std::ofstream m_file;
    std::vector<std::string> m_lines;
    bool m_stopped = false;
};

/// polyfem's default log level (json-specs/log.json: /output/log/level defaults to "debug"). The
/// executable only overrides it when `--log_level` is passed and the Python engine never passes
/// it, so this is the level both engines' log files are written at. Read off the document anyway,
/// so that a JSON that did set it would still produce matching logs.
spdlog::level::level_enum document_log_level(const nlohmann::json& doc)
{
    const auto output = doc.find("output");
    if (output != doc.end() && output->is_object()) {
        const auto log = output->find("log");
        if (log != output->end() && log->is_object()) {
            const auto level = log->find("level");
            if (level != log->end() && level->is_string()) {
                return spdlog::level::from_str(level->get<std::string>());
            }
        }
    }
    return spdlog::level::debug;
}

/// Put `sink` on `logger` unless it is already there. polyfem's logger, ipc's logger and (through
/// GeogramUtils) geogram's output are the three things the executable prints on stdout; they are
/// two distinct logger objects sharing one set of sinks, so both have to be fed.
void attach_sink(spdlog::logger& logger, const spdlog::sink_ptr& sink)
{
    auto& sinks = logger.sinks();
    if (std::find(sinks.begin(), sinks.end(), sink) == sinks.end()) {
        sinks.push_back(sink);
    }
}

void detach_sink(spdlog::logger& logger, const spdlog::sink_ptr& sink)
{
    auto& sinks = logger.sinks();
    sinks.erase(std::remove(sinks.begin(), sinks.end(), sink), sinks.end());
}

/// `main.cpp`'s `load_json`: the document, with `root_path` defaulted to the file it came from so
/// that relative paths inside it resolve the same way.
nlohmann::json load_simulation_json(const std::filesystem::path& json_path)
{
    std::ifstream file(json_path);
    if (!file.is_open()) {
        log_and_throw_error("unable to open {} file", json_path.string());
    }
    nlohmann::json out;
    file >> out;
    if (!out.contains("root_path")) {
        out["root_path"] = json_path.string();
    }
    return out;
}

/// The content `inputs` holds under the path `name`, which the simulation JSON gives as `what`.
/// There is no fallback to the file: a missing entry is a path the operation named without
/// generating its content, and reading whatever is on disk there instead could only hide that.
template <typename Content>
const Content& named_content(
    const std::map<std::string, Content>& contents,
    const nlohmann::json& name,
    const std::string& what)
{
    const std::string path = name.get<std::string>();
    const auto it = contents.find(path);
    if (it == contents.end()) {
        log_and_throw_error(
            "the simulation JSON names {} as {}, but its content is not in memory",
            path,
            what);
    }
    return it->second;
}

/// What SolveData's file reader reads out of a constraint file (`read_constraint_file` in polyfem
/// SolveData.cpp), from the arrays that file is written from.
polyfem::solver::ConstraintData constraint_data(const ConstraintHdf5& c)
{
    polyfem::solver::ConstraintData d;
    d.local2global.assign(c.local2global.begin(), c.local2global.end());
    d.A.rows.assign(c.a.rows.begin(), c.a.rows.end());
    d.A.cols.assign(c.a.cols.begin(), c.a.cols.end());
    d.A.values = c.a.values;
    d.A.shape = {long(c.shape[0]), long(c.shape[1])};
    // The file stores b row-major, and h5pp reads it back into a column-major matrix entry for
    // entry, so b(i, j) is the file's row i, column j.
    d.b.resize(c.b_rows, c.b_cols);
    for (int64_t i = 0; i < c.b_rows; ++i) {
        for (int64_t j = 0; j < c.b_cols; ++j) {
            d.b(i, j) = c.b[size_t(i * c.b_cols + j)];
        }
    }
    return d;
}

/**
 * @brief The collision proxy `State::build_collision_mesh` reads out of its three files, from the
 * content those files are written from.
 *
 * The mesh is what `read_surface_mesh` returns for the OBJ. `OBJReader` keeps all three numbers of
 * a "v" line, and the third column, 0 in 2D, is dropped later by the code both routes share; the
 * "f" and "l" lists become matrices through `igl::list_to_matrix`, which turns an empty list into
 * a 0 x 0 matrix; and `find_codim_vertices` marks every vertex that no edge and no face uses.
 * The vertex coordinates need no parse: the OBJ prints each one as the shortest decimal that reads
 * back as the same double, so the file and this content carry the same values (compared exactly,
 * on coordinates that are not integers, in tests/test_polyfem_in_process.cpp).
 */
polyfem::mesh::CollisionProxyData collision_proxy(
    const CollisionObj& obj,
    const LinearMapHdf5& map,
    const std::vector<std::vector<int64_t>>* body_ids)
{
    polyfem::mesh::CollisionProxyData p;
    p.vertices.resize(Eigen::Index(obj.vertices.size()), 3);
    for (size_t i = 0; i < obj.vertices.size(); ++i) {
        for (int d = 0; d < 3; ++d) {
            p.vertices(Eigen::Index(i), d) = obj.vertices[i][size_t(d)];
        }
    }
    if (!obj.faces.empty()) {
        p.faces.resize(Eigen::Index(obj.faces.size()), 3);
        for (size_t i = 0; i < obj.faces.size(); ++i) {
            for (int k = 0; k < 3; ++k) p.faces(Eigen::Index(i), k) = int(obj.faces[i][size_t(k)]);
        }
    }
    if (!obj.edges.empty()) {
        p.codim_edges.resize(Eigen::Index(obj.edges.size()), 2);
        for (size_t i = 0; i < obj.edges.size(); ++i) {
            for (int k = 0; k < 2; ++k) {
                p.codim_edges(Eigen::Index(i), k) = int(obj.edges[i][size_t(k)]);
            }
        }
    }
    std::vector<bool> is_codim(obj.vertices.size(), true);
    for (const auto& f : obj.faces) {
        for (const int64_t v : f) is_codim[size_t(v)] = false;
    }
    for (const auto& e : obj.edges) {
        for (const int64_t v : e) is_codim[size_t(v)] = false;
    }
    std::vector<int> codim;
    for (size_t v = 0; v < is_codim.size(); ++v) {
        if (is_codim[v]) codim.push_back(int(v));
    }
    p.codim_vertices = Eigen::Map<const Eigen::VectorXi>(codim.data(), Eigen::Index(codim.size()));

    p.weight_values =
        Eigen::Map<const Eigen::VectorXd>(map.values.data(), Eigen::Index(map.values.size()));
    p.weight_rows = Eigen::Map<const Eigen::Matrix<int32_t, Eigen::Dynamic, 1>>(
                        map.rows.data(),
                        Eigen::Index(map.rows.size()))
                        .cast<int>();
    p.weight_cols = Eigen::Map<const Eigen::Matrix<int32_t, Eigen::Dynamic, 1>>(
                        map.cols.data(),
                        Eigen::Index(map.cols.size()))
                        .cast<int>();
    p.weight_shape = {long(map.shape[0]), long(map.shape[1])};

    if (body_ids != nullptr) {
        for (const auto& ids : *body_ids) {
            p.collision_body_ids.emplace_back(ids.begin(), ids.end());
        }
    }
    return p;
}

/**
 * @brief The reduced mesh as `polyfem::mesh::Mesh::create(path)` reads it out of the file saved
 * from `spec`: `MshReader::load`'s vertices, cells and body ids, for the only cells the reduced
 * mesh has (linear triangles, or linear tetrahedra).
 *
 * `Mesh::create(path)` then also attaches the higher-order nodes and the cell weights of the
 * file. That step is not repeated here, and the one trace it leaves is `Mesh::orders()`: all ones
 * from the file on a 3D mesh, empty from `Mesh::create(V, F)`. Every element is linear, so an
 * empty order list and a list of ones build the same basis, and the State comparison in
 * tests/test_polyfem_in_process.cpp finds no other difference.
 */
std::unique_ptr<polyfem::mesh::Mesh> reduced_mesh(const mshio::MshSpec& spec)
{
    const int n_vertices = int(spec.nodes.num_nodes);
    const int max_tag = int(spec.nodes.max_node_tag);
    int dim = -1;
    for (const auto& block : spec.elements.entity_blocks) {
        dim = std::max(dim, block.entity_dim);
    }

    // Node placement is MshReader's own rule: the gmsh tag minus one when the tags are exactly
    // 1..n, otherwise the order of appearance, with the same warning.
    if (n_vertices != max_tag) {
        polyfem::logger().warn(
            "MSH file contains more node tags than nodes, condensing nodes which will break input "
            "node ordering.");
    }
    Eigen::MatrixXd vertices(n_vertices, dim);
    std::vector<int> tag_to_index(size_t(max_tag) + 1, -1);
    int index = 0;
    for (const auto& block : spec.nodes.entity_blocks) {
        for (size_t i = 0; i < block.num_nodes_in_block; ++i) {
            const int node_id = n_vertices != max_tag ? index++ : int(block.tags[i]) - 1;
            for (int d = 0; d < dim; ++d) vertices(node_id, d) = block.data[3 * i + size_t(d)];
            tag_to_index[block.tags[i]] = node_id;
        }
    }

    // Each entity's body id is its first physical group, 0 when it has none.
    std::unordered_map<int, int> entity_to_body;
    const auto map_entities = [&entity_to_body](const auto& entities) {
        for (const auto& e : entities) {
            entity_to_body[e.tag] =
                e.physical_group_tags.empty() ? 0 : e.physical_group_tags.front();
        }
    };
    if (dim == 2) {
        map_entities(spec.entities.surfaces);
    } else {
        map_entities(spec.entities.volumes);
    }

    size_t n_cells = 0;
    for (const auto& block : spec.elements.entity_blocks) {
        if (block.entity_dim == dim) n_cells += block.num_elements_in_block;
    }
    const int cols = dim + 1;
    Eigen::MatrixXi cells(Eigen::Index(n_cells), cols);
    std::vector<int> body_ids(n_cells);
    Eigen::Index c = 0;
    for (const auto& block : spec.elements.entity_blocks) {
        if (block.entity_dim != dim) continue;
        const size_t stride = mshio::nodes_per_element(block.element_type) + 1;
        const auto body = entity_to_body.find(block.entity_tag);
        for (size_t j = 0; j < block.num_elements_in_block; ++j, ++c) {
            for (int k = 0; k < cols; ++k) {
                cells(c, k) = tag_to_index[block.data[j * stride + 1 + size_t(k)]];
            }
            body_ids[size_t(c)] = body != entity_to_body.end() ? body->second : 0;
        }
    }

    std::unique_ptr<polyfem::mesh::Mesh> mesh =
        polyfem::mesh::Mesh::create(vertices, cells, /*non_conforming=*/false);
    mesh->set_body_ids(body_ids);
    return mesh;
}

/**
 * @brief The reduced mesh with the geometry entry `j_mesh` applied, as polyfem's `read_fem_mesh`
 * (src/polyfem/mesh/GeometryReader.cpp) applies it to the mesh it loads from the file, done with
 * polyfem's own public functions.
 *
 * `j_mesh` is the entry as State::init filled it in from polyfem's input spec. An operation writes
 * only its mesh and a `{"scale": ...}` transformation, and on such an entry the one step of
 * read_fem_mesh that changes the mesh is the affine transformation. It is repeated here in
 * read_fem_mesh's order and arithmetic: the bounding box of the untransformed mesh, the unit
 * scale, `construct_affine_transformation`, `Mesh::apply_affine_transformation`.
 *
 * Every other step read_fem_mesh (and read_fem_geometry, which calls it) can take is checked to
 * do nothing on `j_mesh`, and a key that asks for more throws with its name: skipping it would
 * build a mesh the file route does not. `advanced.refinement_location` is read only when n_refs
 * is positive, so it needs no check.
 */
std::unique_ptr<polyfem::mesh::Mesh> fem_mesh(
    const polyfem::Units& units,
    const nlohmann::json& j_mesh,
    const mshio::MshSpec& spec)
{
    const auto refuse = [](const std::string& key) {
        log_and_throw_error(
            "geometry[0].{} asks polyfem's geometry reader for a step the in-memory mesh does "
            "not reproduce",
            key);
    };
    // read_fem_geometry: the one entry must be a plain mesh that is part of the FE mesh.
    if (j_mesh.at("type") != "mesh") refuse("type");
    if (!j_mesh.at("enabled").get<bool>()) refuse("enabled");
    if (j_mesh.at("is_obstacle").get<bool>()) refuse("is_obstacle");
    // read_fem_mesh, in its order.
    if (j_mesh.at("extract") != "volume") refuse("extract");
    if (j_mesh.at("advanced").at("normalize_mesh").get<bool>()) refuse("advanced.normalize_mesh");
    if (j_mesh.at("n_refs").get<int>() > 0) refuse("n_refs");
    if (j_mesh.at("advanced").at("min_component").get<int>() != -1) {
        refuse("advanced.min_component");
    }
    if (j_mesh.at("advanced").at("force_linear_geometry").get<bool>()) {
        refuse("advanced.force_linear_geometry");
    }
    if (polyfem::utils::is_param_valid(j_mesh, "point_selection")) refuse("point_selection");
    if (!j_mesh.at("curve_selection").is_null()) refuse("curve_selection");
    if (polyfem::utils::is_param_valid(j_mesh, "surface_selection")) refuse("surface_selection");
    // State::init fills an absent volume_selection as {"id_offset": 0} (measured on every case of
    // tests/test_polyfem_in_process.cpp). read_fem_mesh sends that form to its id_offset branch,
    // which at offset 0 leaves every body id as the mesh stores it, so skipping it is exact. Its
    // other branch -- compute_body_ids over the listed selections, with the mesh's stored ids
    // appended as the lowest priority -- is taken for any other volume_selection, refused here.
    if (j_mesh.at("volume_selection") != nlohmann::json::object({{"id_offset", 0}})) {
        refuse("volume_selection");
    }

    std::unique_ptr<polyfem::mesh::Mesh> mesh = reduced_mesh(spec);

    polyfem::RowVectorNd min, max;
    mesh->bounding_box(min, max);

    const std::string unit = j_mesh.at("unit");
    double unit_scale = 1;
    if (!unit.empty()) {
        unit_scale = polyfem::Units::convert(1, unit, units.length());
    }

    polyfem::MatrixNd A;
    polyfem::VectorNd b;
    polyfem::mesh::construct_affine_transformation(
        unit_scale,
        j_mesh.at("transformation"),
        (max - min).cwiseAbs().transpose(),
        A,
        b);
    mesh->apply_affine_transformation(A, b);
    return mesh;
}

std::optional<double> active_distance_from_contact_form(
    const polyfem::State& state,
    const Eigen::MatrixXd& sol)
{
    const auto* form = dynamic_cast<const polyfem::solver::SmoothContactForm*>(
        state.solve_data.contact_form.get());
    if (form == nullptr) {
        // No contact at all (the smoothing operation), or a formulation whose post_step does not
        // log the line: either way the executable's log has no line for the Python to parse.
        return std::nullopt;
    }
    const Eigen::MatrixXd displaced = form->compute_displaced_surface(sol.col(0));
    const double minimum =
        form->collision_set().compute_minimum_distance(state.collision_mesh, displaced);
    if (std::isinf(minimum)) {
        // SmoothContactForm::post_step's own guard: with nothing within dhat it prints no line.
        return std::nullopt;
    }
    // post_step prints sqrt() of the squared distance the collision set reports.
    return std::sqrt(
        form->collision_set().compute_active_minimum_distance(state.collision_mesh, displaced));
}

/**
 * @brief The in-process backend: `src/polyfem/main.cpp`'s `forward_simulation`, call for call, on
 * a State built here.
 *
 * The simulation JSON is byte for byte the one the Python engine hands the executable, and it is
 * still what says which inputs a solve reads, but no input is read from a file: the reduced mesh,
 * the soft constraints, the hard pins and the collision proxy with its linear map and body ids all
 * come out of `m_inputs`, under the paths the JSON names them by (`prepare_state`).
 *
 * Nor does the warm start go through a file: the JSON's `input/data/state` and
 * `output/data/state` are blanked in the in-memory copy of the arguments, and the three matrices
 * polyfem would have written to `curr_state.hdf5` are carried in `m_last` instead.
 */
class InProcessBackend : public PolyfemBackend
{
public:
    explicit InProcessBackend(SolveInputs inputs)
        : m_inputs(std::move(inputs))
    {}

    SolveResult solve(
        const std::filesystem::path& json_path,
        const std::filesystem::path& out_dir,
        const std::filesystem::path& log_path) override
    {
        nlohmann::json args = load_simulation_json(json_path);

        // The loop puts prev_state.hdf5 into the document exactly when it has committed a solve,
        // which is exactly when m_committed holds one; the file itself is never written or read.
        const bool wants_warm_start =
            args.contains("input") && args["input"].contains("data") &&
            args["input"]["data"].contains("state") &&
            !args["input"]["data"]["state"].get<std::string>().empty();
        if (wants_warm_start && !m_committed.has_value()) {
            log_and_throw_error(
                "{} asks polyfem to warm start from {}, but no solve has been committed in this "
                "process",
                json_path.string(),
                args["input"]["data"]["state"].get<std::string>());
        }
        if (args.contains("input") && args["input"].contains("data")) {
            args["input"]["data"]["state"] = "";
        }
        if (args.contains("output") && args["output"].contains("data")) {
            args["output"]["data"]["state"] = "";
        }

        // `-o <out_dir>`, the one command-line argument the Python engine passes the executable
        // besides `-j`, applied the way main.cpp applies it.
        nlohmann::json patch = nlohmann::json::object();
        patch["/output/directory"_json_pointer] = std::filesystem::absolute(out_dir).string();
        args.merge_patch(patch);

        if (!log_path.parent_path().empty()) {
            std::filesystem::create_directories(log_path.parent_path());
        }
        auto capture = std::make_shared<LogCapture>(log_path);
        capture->set_level(document_log_level(args));
        // Attached BEFORE State::init, which logs (the linear-solver choice, among others) before
        // it installs its own logger, and again after, because init replaces the logger object.
        attach_sink(polyfem::logger(), capture);
        attach_sink(ipc::logger(), capture);

        SolveResult result;
        try {
            result = run_solve(args, capture, wants_warm_start);
        } catch (const std::exception& e) {
            // The executable dies on this (an uncaught exception aborts main, so the Python engine
            // sees return code -6); in process there is no signal to report, so the failure is a
            // non-zero code and check_polyfem_success prints its banner as usual.
            // polyfem has already logged the message itself -- log_and_throw_error logs before it
            // throws -- so it is in the captured lines and in the log file.
            logger().error("polyfem failed in process: {}", e.what());
            result.returncode = 1;
            m_last.reset();
        }

        detach_sink(polyfem::logger(), capture);
        detach_sink(ipc::logger(), capture);
        capture->flush();
        result.lines = capture->lines();
        capture->stop();
        return result;
    }

    void reset_warm_start() override
    {
        // Nothing on disk to unlink, only the two solutions to drop.
        m_last.reset();
        m_committed.reset();
    }

    void commit_warm_start() override { m_committed = m_last; }

    bool has_warm_start() const override { return m_committed.has_value(); }

private:
    /// main.cpp's `forward_simulation` for a JSON input, with the inputs and the initial
    /// condition handed over in memory. Returns the active distance and the subsolve statuses,
    /// both read off the State before it is destroyed; `returncode` and `lines` are the caller's.
    SolveResult run_solve(
        const nlohmann::json& args,
        const std::shared_ptr<LogCapture>& capture,
        const bool wants_warm_start)
    {
        polyfem::State state;
        prepare_state(state, args, m_inputs, capture);

        Eigen::MatrixXd sol;
        Eigen::MatrixXd pressure;

        polyfem::InitialConditionOverride initial_condition;
        if (wants_warm_start) {
            initial_condition.solution = m_committed->solution;
            initial_condition.velocity = m_committed->velocity;
            initial_condition.acceleration = m_committed->acceleration;
        }
        state.solve_problem(sol, pressure, {}, wants_warm_start ? &initial_condition : nullptr);

        state.compute_errors(sol);

        polyfem::logger().info("total time: {}s", state.timings.total_time());

        state.save_json(sol);
        state.export_data(sol, pressure);

        // What polyfem would have written to curr_state.hdf5 at this point: the transient loop
        // calls save_state after update_quantities, so x_prevs() already holds this solve's
        // solution.
        m_last.reset();
        if (state.solve_data.time_integrator != nullptr) {
            const auto& integrator = *state.solve_data.time_integrator;
            const int ndof = int(integrator.x_prev().size());
            const int prev_steps = int(integrator.x_prevs().size());
            SolverState next;
            next.solution.resize(ndof, prev_steps);
            next.velocity.resize(ndof, prev_steps);
            next.acceleration.resize(ndof, prev_steps);
            for (int i = 0; i < prev_steps; ++i) {
                next.solution.col(i) = integrator.x_prevs()[i];
                next.velocity.col(i) = integrator.v_prevs()[i];
                next.acceleration.col(i) = integrator.a_prevs()[i];
            }
            m_last = std::move(next);
        }

        SolveResult result;
        result.active_distance = active_distance_from_contact_form(state, sol);
        // One entry per AL, reduced and lagging subsolve; an entry lacks a status only when no
        // solver ran (polyfem's ALSolver::record_solver_info).
        for (const auto& entry : state.stats.solver_info) {
            const auto& info = entry.at("info");
            if (info.contains("status")) {
                result.statuses.push_back(info.at("status").get<polysolve::nonlinear::Status>());
            }
        }
        return result;
    }

    const SolveInputs m_inputs; ///< every input file the simulation JSON names
    std::optional<SolverState> m_last; ///< polyfem's curr_state.hdf5
    std::optional<SolverState> m_committed; ///< polyfem's prev_state.hdf5
};

} // namespace

void prepare_state(
    polyfem::State& state,
    const nlohmann::json& args,
    const SolveInputs& inputs,
    const spdlog::sink_ptr& log_sink)
{
    // Every input the document names, looked up under the name the document gives it. The copy
    // polyfem is initialised with names none of them: the constraint lists are emptied (the key
    // itself is kept, because State::init decides has_constraints() on its presence) and the
    // collision mesh is reduced to "enabled", which is how polyfem is told the proxy is in memory.
    nlohmann::json polyfem_args = args;
    const nlohmann::json& geometry = args.at("geometry");
    if (!geometry.is_array() || geometry.size() != 1) {
        log_and_throw_error("the simulation JSON must have exactly one geometry entry");
    }
    const mshio::MshSpec& msh =
        named_content(inputs.meshes, geometry[0].at("mesh"), "geometry[0].mesh");

    std::vector<polyfem::solver::ConstraintData> hard;
    std::vector<polyfem::solver::ConstraintData> soft;
    if (args.contains("constraints")) {
        const nlohmann::json& constraints = args["constraints"];
        if (constraints.contains("hard")) {
            for (const auto& path : constraints["hard"]) {
                hard.push_back(
                    constraint_data(named_content(inputs.constraints, path, "constraints.hard")));
            }
            polyfem_args["constraints"]["hard"] = nlohmann::json::array();
        }
        if (constraints.contains("soft")) {
            for (const auto& entry : constraints["soft"]) {
                soft.push_back(constraint_data(named_content(
                    inputs.constraints,
                    entry.at("data"),
                    "constraints.soft[*].data")));
                soft.back().weight = entry.at("weight").get<double>();
            }
            polyfem_args["constraints"]["soft"] = nlohmann::json::array();
        }
    }

    // The proxy's files are read exactly when polyfem's file route reads them: a collision mesh
    // that is enabled (the spec's default) and names a mesh.
    std::optional<polyfem::mesh::CollisionProxyData> proxy;
    const nlohmann::json::json_pointer collision_mesh_ptr("/contact/collision_mesh");
    if (args.contains(collision_mesh_ptr) && args.at(collision_mesh_ptr).value("enabled", true) &&
        args.at(collision_mesh_ptr).contains("mesh")) {
        const nlohmann::json& collision_mesh = args.at(collision_mesh_ptr);
        const auto* body_ids = collision_mesh.contains("collision_body_ids")
                                   ? &named_content(
                                         inputs.collision_body_ids,
                                         collision_mesh["collision_body_ids"],
                                         "contact.collision_mesh.collision_body_ids")
                                   : nullptr;
        proxy = collision_proxy(
            named_content(
                inputs.collision_meshes,
                collision_mesh["mesh"],
                "contact.collision_mesh.mesh"),
            named_content(
                inputs.linear_maps,
                collision_mesh.at("linear_map"),
                "contact.collision_mesh.linear_map"),
            body_ids);
        polyfem_args[collision_mesh_ptr] = {{"enabled", true}};
    }

    // State::init's validation does not need the named files to exist: jse checks a "file" rule
    // against the disk only when its skip_file_check is off, and it is on by default.
    state.init(polyfem_args, /*strict_validation=*/true);
    if (log_sink != nullptr) {
        attach_sink(polyfem::logger(), log_sink);
        attach_sink(ipc::logger(), log_sink);
    }
    state.in_memory_hard_constraints = std::move(hard);
    state.in_memory_soft_constraints = std::move(soft);
    state.in_memory_collision_proxy = std::move(proxy);

    // `read_fem_geometry` for the one mesh, from memory: `read_fem_mesh` on the geometry entry as
    // State::init filled it in (`fem_mesh`), and `read_fem_geometry` keeps a COPY of the first
    // mesh. The copy is not a formality: a 2D mesh's copy() rebuilds it and orients its elements
    // again. Measured on a 2D mesh under the mirroring scale [-1e-3, 1e-3]: from the file it
    // loads, from memory without the copy it is refused with "element 0 is flipped".
    // State::load_mesh keeps a mesh that is already set.
    state.mesh =
        fem_mesh(state.units, polyfem::utils::json_as_array(state.args["geometry"])[0], msh)
            ->copy();
    state.load_mesh(/*non_conforming=*/false, {}, {}, {});
    state.stats.compute_mesh_stats(*state.mesh);

    state.build_basis();

    state.assemble_rhs();
    state.assemble_mass_mat();
}

std::unique_ptr<PolyfemBackend> in_process_backend(SolveInputs inputs)
{
    return std::make_unique<InProcessBackend>(std::move(inputs));
}

} // namespace wmtk::components::polyfem_ops
