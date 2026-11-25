// PolyFEM in process (WMTK_WITH_POLYFEM): polyfem_simulate runs a simulation as PolyFEM_bin does
// (polyfem's src/polyfem/main.cpp, run_forward_simulation), with the meshes in memory instead of
// in files. polyfem reads every input through a ResourceIO; MemoryIO is one whose resources are
// the meshes the caller passes, as polyfem's mesh groups (MeshLoader::load_fem): `vertices`,
// `cells` and, optionally, `body_ids`, `boundary_elements` and `boundary_ids`.

#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <nlohmann/json.hpp>
#include <pybind11_json/pybind11_json.hpp>

#include <spdlog/sinks/ostream_sink.h>
#include <polyfem/State.hpp>
#include <polyfem/autogen/auto_p_bases.hpp>
#include <polyfem/io/OutputData.hpp>
#include <polyfem/io/ResourceIO.hpp>
#include <polyfem/mesh/MeshLoader.hpp>
#include <polyfem/utils/Logger.hpp>
#include <polyfem/varforms/VarForm.hpp>

#include <algorithm>
#include <array>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>

#include "mesh_conversion.hpp"

namespace py = pybind11;
using namespace pybind11::literals;

namespace wmtk::python {
namespace {

// Groups, datasets and text files under logical paths ("mesh/vertices"), as HDF5IO has them:
// integer datasets are not read as floating point and floating point not as integers.
class MemoryIO final : public polyfem::io::ResourceIO
{
public:
    using Dataset = std::variant<std::monostate, Eigen::MatrixXd, Eigen::MatrixXi, std::string>;
    struct Node
    {
        Dataset data; // monostate: a group
        std::map<std::string, long> integer_attributes;
        std::map<std::string, std::string> string_attributes;
        std::map<std::string, std::array<long, 2>> shape_attributes;
        bool group() const { return std::holds_alternative<std::monostate>(data); }
    };
    using Tree = std::map<std::string, Node>;

    explicit MemoryIO(std::shared_ptr<const Tree> tree, std::string root = {})
        : m_tree(std::move(tree))
        , m_root(std::move(root))
        , m_host_directory(std::filesystem::current_path())
    {}

    // The key of a logical path: normalized, relative to the tree's root, no leading "/" or "./".
    static std::string normalize(const std::string& path)
    {
        std::string k = std::filesystem::path(path).lexically_normal().generic_string();
        while (!k.empty() && (k.front() == '/' || k.rfind("./", 0) == 0))
            k.erase(0, k.front() == '/' ? 1 : 2);
        if (k == ".") k.clear();
        while (!k.empty() && k.back() == '/') k.pop_back();
        return k;
    }

    std::unique_ptr<const ResourceIO> with_root(const std::string& root) const override
    {
        auto result = std::make_unique<MemoryIO>(m_tree, key(root));
        result->access_tracker_ = access_tracker_;
        return result;
    }
    std::string canonical_path(const std::string& path) const override { return "/" + key(path); }
    bool exists(const std::string& path) const override
    {
        const auto k = key(path);
        return k.empty() || m_tree->count(k);
    }
    bool is_group(const std::string& path) const override
    {
        const auto k = key(path);
        return k.empty() || (m_tree->count(k) && m_tree->at(k).group());
    }
    std::vector<std::string> list(const std::string& path) const override
    {
        const auto k = key(path);
        std::vector<std::string> result;
        for (const auto& [name, node] : *m_tree) {
            const auto parent = std::filesystem::path(name).parent_path().generic_string();
            if (parent == k)
                result.push_back(
                    (std::filesystem::path(path) / std::filesystem::path(name).filename())
                        .lexically_normal()
                        .generic_string());
        }
        return result;
    }
    std::unique_ptr<std::istream> open(const std::string& path, bool) const override
    {
        if (const auto* text = std::get_if<std::string>(&node(path).data))
            return std::make_unique<std::istringstream>(*text);
        throw std::runtime_error(describe(path) + " is not a text resource");
    }
    Eigen::MatrixXd read_matrix(const std::string& path) const override
    {
        const auto& data = dataset(path);
        if (const auto* d = std::get_if<Eigen::MatrixXd>(&data)) return *d;
        if (const auto* i = std::get_if<Eigen::MatrixXi>(&data)) return i->cast<double>();
        throw std::runtime_error(describe(path) + " is text, not a matrix");
    }
    Eigen::MatrixXi read_int_matrix(const std::string& path) const override
    {
        const auto& data = dataset(path);
        if (const auto* i = std::get_if<Eigen::MatrixXi>(&data)) return *i;
        throw std::runtime_error(describe(path) + " does not hold integers");
    }
    std::vector<double> read_double_vector(const std::string& path) const override
    {
        const Eigen::MatrixXd m = read_matrix(path);
        require_vector(path, m.rows(), m.cols());
        return {m.data(), m.data() + m.size()};
    }
    std::vector<int> read_int_vector(const std::string& path) const override
    {
        const Eigen::MatrixXi m = read_int_matrix(path);
        require_vector(path, m.rows(), m.cols());
        return {m.data(), m.data() + m.size()};
    }
    std::vector<long> read_long_vector(const std::string& path) const override
    {
        const std::vector<int> v = read_int_vector(path);
        return {v.begin(), v.end()};
    }
    bool has_attribute(const std::string& path, const std::string& name) const override
    {
        const auto it = m_tree->find(key(path));
        return it != m_tree->end() && (it->second.integer_attributes.count(name) ||
                                       it->second.string_attributes.count(name) ||
                                       it->second.shape_attributes.count(name));
    }
    long read_integer_attribute(const std::string& path, const std::string& name) const override
    {
        return attribute(node(path).integer_attributes, path, name);
    }
    std::string read_string_attribute(const std::string& path, const std::string& name)
        const override
    {
        return attribute(node(path).string_attributes, path, name);
    }
    std::array<long, 2> read_shape_attribute(const std::string& path, const std::string& name)
        const override
    {
        return attribute(node(path).shape_attributes, path, name);
    }
    std::filesystem::path materialize(const std::string& path) const override
    {
        throw std::runtime_error(describe(path) + " is in memory and has no file");
    }
    const std::filesystem::path& host_directory() const override { return m_host_directory; }
    std::string describe(const std::string& path) const override
    {
        return "in-memory resource '" + key(path) + "'";
    }

private:
    std::string key(const std::string& path) const
    {
        return normalize((std::filesystem::path(m_root) / path).generic_string());
    }
    const Node& node(const std::string& path) const
    {
        const auto it = m_tree->find(key(path));
        if (it == m_tree->end()) throw std::runtime_error(describe(path) + " does not exist");
        record_access(path);
        return it->second;
    }
    const Dataset& dataset(const std::string& path) const
    {
        const Node& n = node(path);
        if (n.group()) throw std::runtime_error(describe(path) + " is a group, not a dataset");
        return n.data;
    }
    void require_vector(const std::string& path, Eigen::Index rows, Eigen::Index cols) const
    {
        if (rows != 1 && cols != 1)
            throw std::runtime_error(describe(path) + " is a matrix, not a vector");
    }
    template <typename T>
    T attribute(
        const std::map<std::string, T>& attributes,
        const std::string& path,
        const std::string& name) const
    {
        const auto it = attributes.find(name);
        if (it == attributes.end())
            throw std::runtime_error(describe(path) + " has no attribute " + name);
        return it->second;
    }

    std::shared_ptr<const Tree> m_tree;
    std::string m_root;
    std::filesystem::path m_host_directory;
};

// The tree of `resources` ({path: NumPy array or str}) and `attributes` ({path: {name: int, str
// or [rows, cols]}}). Every path's parents are groups, and so is an attributes path that is not a
// resource.
MemoryIO::Tree resource_tree(const py::dict& resources, const py::dict& attributes)
{
    MemoryIO::Tree tree;
    const auto add_parents = [&](const std::string& k) {
        for (auto p = std::filesystem::path(k).parent_path(); !p.empty(); p = p.parent_path())
            tree.emplace(p.generic_string(), MemoryIO::Node{});
    };
    for (auto item : resources) {
        const std::string k = MemoryIO::normalize(py::cast<std::string>(item.first));
        if (k.empty()) throw py::value_error("polyfem_simulate: a resource needs a path");
        MemoryIO::Node node;
        if (py::isinstance<py::str>(item.second))
            node.data = py::cast<std::string>(item.second);
        else {
            const py::array values = array(item.second);
            const char kind = values.dtype().kind();
            if (kind == 'f')
                node.data = py::cast<Eigen::MatrixXd>(values);
            else if (kind == 'i' || kind == 'u' || kind == 'b')
                node.data = py::cast<Eigen::MatrixXi>(values.attr("astype")("int32"));
            else
                throw py::type_error("polyfem_simulate: resource '" + k + "' must be numeric");
        }
        add_parents(k);
        tree[k] = std::move(node);
    }
    for (auto item : attributes) {
        const std::string k = MemoryIO::normalize(py::cast<std::string>(item.first));
        add_parents(k);
        MemoryIO::Node& node = tree[k];
        for (auto attribute : dictionary(item.second, "Attributes")) {
            const auto name = py::cast<std::string>(attribute.first);
            if (py::isinstance<py::str>(attribute.second))
                node.string_attributes[name] = py::cast<std::string>(attribute.second);
            else if (py::isinstance<py::int_>(attribute.second))
                node.integer_attributes[name] = py::cast<long>(attribute.second);
            else
                node.shape_attributes[name] = py::cast<std::array<long, 2>>(attribute.second);
        }
    }
    return tree;
}

// The displacement at the vertices of polyfem's mesh, from a solution: polyfem's own
// "displacement" output field, sampled at each vertex's corner of the first element that has it
// (zero at a vertex in no element). The corner is found by polyfem's geometric map of its P1
// nodes, so no vertex order is assumed.
class VertexSampler
{
public:
    explicit VertexSampler(const polyfem::varform::VarForm& form)
        : m_form(form)
    {
        const polyfem::io::OutputSpace space = form.output_space();
        if (space.mesh == nullptr || space.geometry_bases == nullptr)
            throw std::runtime_error("polyfem_simulate: polyfem has no mesh to sample");
        const polyfem::mesh::Mesh& mesh = *space.mesh;
        if (!mesh.is_simplicial())
            throw std::runtime_error("polyfem_simulate: only simplicial meshes are supported");
        const int dim = mesh.dimension();
        Eigen::MatrixXd corners;
        if (dim == 3)
            polyfem::autogen::p_nodes_3d(1, corners);
        else
            polyfem::autogen::p_nodes_2d(1, corners);

        const int n = mesh.n_vertices();
        m_sample.points.setZero(n, dim);
        m_sample.local_points.setZero(n, dim);
        m_sample.element_ids.setConstant(n, -1);
        m_sample.domain = polyfem::io::OutputSample::Domain::Volume;
        for (int e = 0; e < mesh.n_elements(); ++e) {
            Eigen::MatrixXd mapped;
            (*space.geometry_bases)[e].eval_geom_mapping(corners, mapped);
            for (int j = 0; j < corners.rows(); ++j) {
                int best = -1;
                double best_distance = std::numeric_limits<double>::infinity();
                for (int lv = 0; lv < corners.rows(); ++lv) {
                    const int v = mesh.element_vertex(e, lv);
                    const double d = (mesh.point(v) - mapped.row(j)).squaredNorm();
                    if (d < best_distance) {
                        best_distance = d;
                        best = v;
                    }
                }
                if (m_sample.element_ids(best) >= 0) continue;
                m_sample.element_ids(best) = e;
                m_sample.local_points.row(best) = corners.row(j);
                m_sample.points.row(best) = mesh.point(best);
            }
        }
        // A vertex in no element keeps element id -1, which output_fields reads as zero, as
        // polyfem's own solution export in input order (reorder_nodes) leaves it.
        m_options.fields = {"displacement"};
    }

    Eigen::MatrixXd operator()(const Eigen::MatrixXd& solution) const
    {
        for (const auto& field : m_form.output_fields(m_sample, solution, m_options))
            if (field.name == "displacement") return field.values;
        throw std::runtime_error("polyfem_simulate: the formulation has no displacement field");
    }

private:
    const polyfem::varform::VarForm& m_form;
    polyfem::io::OutputSample m_sample;
    polyfem::io::OutputFieldOptions m_options;
};

py::dict polyfem_simulate(
    const py::dict& args,
    const py::dict& resources_in,
    const py::dict& attributes,
    const std::function<void(int, const Eigen::MatrixXd&)>& step_callback,
    const std::optional<Eigen::MatrixXd>& initial_solution,
    const bool strict)
{
    const MemoryIO resources(
        std::make_shared<const MemoryIO::Tree>(resource_tree(resources_in, attributes)));
    const nlohmann::json in_args = options(args);

    Eigen::MatrixXd displacement, solution;
    std::ostringstream log;
    {
        py::gil_scoped_release release;
        polyfem::State state;
        state.init(in_args, resources, strict);

        // polyfem's log, for the caller to read (minimum separation reads the active distance
        // from it, as from PolyFEM_bin's output), as well as where the arguments send it.
        auto sink = std::make_shared<spdlog::sinks::ostream_sink_mt>(log);
        std::vector<spdlog::sink_ptr>& sinks = polyfem::logger().sinks();
        sinks.push_back(sink);
        struct RemoveSink
        {
            std::vector<spdlog::sink_ptr>& sinks;
            spdlog::sink_ptr sink;
            ~RemoveSink()
            {
                sinks.erase(std::remove(sinks.begin(), sinks.end(), sink), sinks.end());
            }
        } remove_sink{sinks, sink};

        state.load_mesh();

        // Built on first use: polyfem builds its FE space in solve (VarForm::prepare), before the
        // first step's callback.
        std::optional<VertexSampler> sampler;
        const auto sample = [&](const Eigen::MatrixXd& s) {
            if (!sampler) sampler.emplace(*state.variational_formulation);
            return (*sampler)(s);
        };
        polyfem::varform::ForwardStepCallback post_step;
        if (step_callback)
            post_step = [&](int step, const Eigen::MatrixXd& s) {
                const Eigen::MatrixXd d = sample(s);
                py::gil_scoped_acquire acquire;
                step_callback(step, d);
            };
        polyfem::varform::InitialConditionOverride initial;
        if (initial_solution) initial.solution = *initial_solution;

        // As run_forward_simulation: State::solve, then the statistics and the outputs the
        // arguments ask for (none unless they name files).
        state.variational_formulation->set_time_callback(state.time_callback);
        state.variational_formulation->solve(
            solution,
            initial_solution ? &initial : nullptr,
            post_step);
        state.variational_formulation->compute_errors(solution);
        state.variational_formulation->save_json(solution);
        state.variational_formulation->export_data(solution);
        displacement = sample(solution);
    }
    std::vector<std::string> lines;
    std::istringstream in(log.str());
    for (std::string line; std::getline(in, line);) lines.push_back(line);
    py::dict result;
    result["displacement"] = displacement;
    result["solution"] = solution;
    result["log"] = lines;
    return result;
}

} // namespace

void bind_polyfem(py::module_& m)
{
    m.attr("POLYFEM_MESH_SCHEMA_VERSION") = polyfem::mesh::MESH_SCHEMA_VERSION;
    m.def(
        "polyfem_simulate",
        &polyfem_simulate,
        "args"_a,
        "resources"_a,
        "attributes"_a = py::dict(),
        "step_callback"_a = nullptr,
        "initial_solution"_a = std::nullopt,
        "strict"_a = true,
        "Run a PolyFEM simulation in this process, as PolyFEM_bin runs it, reading its inputs "
        "from memory instead of files.\n\n"
        "args: the simulation JSON; its paths (a geometry's \"mesh\", a soft constraint's "
        "\"data\", ...) name resources.\n"
        "resources: {path: NumPy array (a dataset) or str (a text file)}; the parents of a "
        "path are groups, as in an HDF5 file PolyFEM reads (a mesh group: <name>/vertices, "
        "<name>/cells, ...).\n"
        "attributes: {path: {name: int, str or [rows, cols]}} (a mesh group's schema_version "
        "-- POLYFEM_MESH_SCHEMA_VERSION --, mesh_type and dimension).\n"
        "step_callback(step, displacement): called after each time step, from 0.\n"
        "initial_solution: the state to start from, a \"solution\" an earlier call on the "
        "same mesh and discretization returned (a warm start).\n"
        "Returns {\"displacement\": the final displacement at the vertices of PolyFEM's mesh "
        "(the geometry entries' vertices, in order), \"solution\": PolyFEM's final solution "
        "(its DOFs, in its order), \"log\": PolyFEM's log lines}. Files are written only "
        "where args name them.");
}

} // namespace wmtk::python
