#include "wildmeshing.hpp"

#include "WildmeshingOptions.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>

#include <wmtk/components/base/get_attributes.hpp>
#include <wmtk/multimesh/consolidate.hpp>
#include <wmtk/utils/Logger.hpp>


#include <wmtk/operations/attribute_new/SplitNewAttributeStrategy.hpp>
#include <wmtk/operations/attribute_update/AttributeTransferStrategy.hpp>

#include <wmtk/operations/AMIPSOptimizationSmoothing.hpp>
#include <wmtk/operations/AndOperationSequence.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/MinOperationSequence.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/operations/OrOperationSequence.hpp>
#include <wmtk/operations/Rounding.hpp>
#include <wmtk/operations/TetWildTangentialLaplacianSmoothing.hpp>
#include <wmtk/operations/composite/ProjectOperation.hpp>
#include <wmtk/operations/composite/TetEdgeSwap.hpp>
#include <wmtk/operations/composite/TetFaceSwap.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>


#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/PerSimplexFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>

#include <wmtk/function/utils/amips.hpp>
#include <wmtk/invariants/CollapseEnergyBeforeInvariantDouble.hpp>
#include <wmtk/invariants/EdgeValenceInvariant.hpp>
#include <wmtk/invariants/EnergyFilterInvariant.hpp>
#include <wmtk/invariants/EnvelopeInvariant.hpp>
#include <wmtk/invariants/FunctionInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorSimplexInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MaxFunctionInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/NoBoundaryCollapseToInteriorInvariant.hpp>
#include <wmtk/invariants/NoChildMeshAttachingInvariant.hpp>
#include <wmtk/invariants/RoundedInvariant.hpp>
#include <wmtk/invariants/SeparateSubstructuresInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/Swap23EnergyBeforeInvariantDouble.hpp>
#include <wmtk/invariants/Swap32EnergyBeforeInvariantDouble.hpp>
#include <wmtk/invariants/Swap44EnergyBeforeInvariantDouble.hpp>
#include <wmtk/invariants/Swap56EnergyBeforeInvariantDouble.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>

#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>

#include <wmtk/utils/Rational.hpp>
#include <wmtk/utils/orient.hpp>

#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>

#include <queue>
#include <wmtk/simplex/k_ring.hpp>
#include <wmtk/simplex/link.hpp>

#include <fstream>
namespace wmtk::components {

using namespace simplex;
using namespace operations;
using namespace operations::tri_mesh;
using namespace operations::tet_mesh;
using namespace operations::composite;
using namespace function;
using namespace invariants;

namespace {
void write(
    const std::shared_ptr<Mesh>& mesh,
    const std::string& out_dir,
    const std::string& name,
    const std::string& vname,
    const int64_t index,
    const bool intermediate_output)
{
    if (intermediate_output) {
        if (mesh->top_simplex_type() == PrimitiveType::Triangle) {
            // write trimesh
            const std::filesystem::path data_dir = "";
            wmtk::io::ParaviewWriter writer(
                data_dir / (name + "_" + std::to_string(index)),
                "vertices",
                *mesh,
                true,
                true,
                true,
                false);
            mesh->serialize(writer);
        } else if (mesh->top_simplex_type() == PrimitiveType::Tetrahedron) {
            // write tetmesh
            const std::filesystem::path data_dir = "";
            wmtk::io::ParaviewWriter writer(
                data_dir / (name + "_" + std::to_string(index)),
                "vertices",
                *mesh,
                true,
                true,
                true,
                true);
            mesh->serialize(writer);
        } else if (mesh->top_simplex_type() == PrimitiveType::Edge) {
            // write edgemesh
            const std::filesystem::path data_dir = "";
            wmtk::io::ParaviewWriter writer(
                data_dir / (name + "_" + std::to_string(index)),
                "vertices",
                *mesh,
                true,
                true,
                false,
                false);
            mesh->serialize(writer);
        }
    }
}

} // namespace

void adjust_sizing_field(
    Mesh& m,
    const TypedAttributeHandle<double>& coordinate_handle,
    const TypedAttributeHandle<double>& edge_length_handle,
    const TypedAttributeHandle<double>& sizing_field_scalar_handle,
    const TypedAttributeHandle<double>& energy_handle,
    const TypedAttributeHandle<double>& target_edge_length_handle,
    const TypedAttributeHandle<char>& visited_handle,
    const double stop_energy,
    const double current_max_energy,
    const double initial_target_edge_length,
    const double min_target_edge_length)
{
    if (m.top_simplex_type() != PrimitiveType::Tetrahedron) return;

    std::cout << "in here" << std::endl;

    const auto coordinate_accessor = m.create_const_accessor<double>(coordinate_handle);
    const auto edge_length_accessor = m.create_const_accessor<double>(edge_length_handle);
    const auto energy_accessor = m.create_const_accessor<double>(energy_handle);

    auto sizing_field_scalar_accessor = m.create_accessor<double>(sizing_field_scalar_handle);
    auto target_edge_length_accessor = m.create_accessor<double>(target_edge_length_handle);
    auto visited_accessor = m.create_accessor<char>(visited_handle);

    const double stop_filter_energy = stop_energy * 0.8;
    double filter_energy = std::max(current_max_energy / 100, stop_filter_energy);
    filter_energy = std::min(filter_energy, 100.0);

    const double recover_scalar = 1.5;
    const double refine_scalar = 0.5;
    const double min_refine_scalar = min_target_edge_length / initial_target_edge_length;

    auto vertices_all = m.get_all(PrimitiveType::Vertex);
    int64_t v_cnt = vertices_all.size();

    std::vector<Vector3d> centroids;
    std::queue<Tuple> v_queue;
    // std::vector<double> scale_multipliers(v_cnt, recover_scalar);

    // get centroids and initial v_queue
    for (const auto& t : m.get_all(PrimitiveType::Tetrahedron)) {
        // if (std::cbrt(energy_accessor.const_scalar_attribute(t)) < filter_energy) {
        if (energy_accessor.const_scalar_attribute(t) < filter_energy) {
            // skip good tets
            continue;
        }
        auto vertices = m.orient_vertices(t);
        Vector3d c(0, 0, 0);
        for (int i = 0; i < 4; ++i) {
            c += coordinate_accessor.const_vector_attribute(vertices[i]);
            v_queue.emplace(vertices[i]);
        }
        centroids.emplace_back(c / 4.0);
    }

    wmtk::logger().info(
        "filter energy: {}, low quality tets num: {}",
        filter_energy,
        centroids.size());

    const double R = initial_target_edge_length * 1.8; // update field radius

    for (const auto& v : vertices_all) { // reset visited flag
        visited_accessor.scalar_attribute(v) = char(0);
    }

    // TODO: use efficient data structure
    auto get_nearest_dist = [&](const Tuple& v) -> double {
        Tuple nearest_tuple;
        double min_dist = std::numeric_limits<double>::max();
        const Vector3d v_pos = coordinate_accessor.const_vector_attribute(v);
        for (const auto& c_pos : centroids) {
            double dist = (c_pos - v_pos).norm();
            min_dist = std::min(min_dist, dist);
        }
        return min_dist;
    };

    while (!v_queue.empty()) {
        auto v = v_queue.front();
        v_queue.pop();

        if (visited_accessor.scalar_attribute(v) == char(1)) continue;
        visited_accessor.scalar_attribute(v) = char(1);

        double dist = std::max(0., get_nearest_dist(v));

        if (dist > R) {
            visited_accessor.scalar_attribute(v) = char(0);
            continue;
        }

        double scale_multiplier = std::min(
            recover_scalar,
            dist / R * (1 - refine_scalar) + refine_scalar); // linear interpolate

        auto new_scale = sizing_field_scalar_accessor.scalar_attribute(v) * scale_multiplier;
        if (new_scale > 1) {
            sizing_field_scalar_accessor.scalar_attribute(v) = 1;
        } else if (new_scale < min_refine_scalar) {
            sizing_field_scalar_accessor.scalar_attribute(v) = min_refine_scalar;
        } else {
            sizing_field_scalar_accessor.scalar_attribute(v) = new_scale;
        }

        // push one ring vertices into the queue
        for (const auto& v_one_ring : simplex::link(m, simplex::Simplex::vertex(m, v))
                                          .simplex_vector(PrimitiveType::Vertex)) {
            if (visited_accessor.scalar_attribute(v_one_ring) == char(1)) continue;
            v_queue.push(v_one_ring.tuple());
        }
    }

    // update the rest
    for (const auto& v : vertices_all) {
        if (visited_accessor.scalar_attribute(v) == char(1)) continue;
        auto new_scale = sizing_field_scalar_accessor.scalar_attribute(v) * 1.5;
        if (new_scale > 1) {
            sizing_field_scalar_accessor.scalar_attribute(v) = 1;
        } else if (new_scale < min_refine_scalar) {
            sizing_field_scalar_accessor.scalar_attribute(v) = min_refine_scalar;
        } else {
            sizing_field_scalar_accessor.scalar_attribute(v) = new_scale;
        }
    }

    // update target edge length
    for (const auto& e : m.get_all(PrimitiveType::Edge)) {
        target_edge_length_accessor.scalar_attribute(e) =
            initial_target_edge_length *
            (sizing_field_scalar_accessor.scalar_attribute(e) +
             sizing_field_scalar_accessor.scalar_attribute(
                 m.switch_tuple(e, PrimitiveType::Vertex))) /
            2;
    }
}

void set_operation_energy_filter(
    Mesh& m,
    const TypedAttributeHandle<double>& coordinate_handle,
    const TypedAttributeHandle<double>& energy_handle,
    const TypedAttributeHandle<char>& energy_filter_handle,
    const TypedAttributeHandle<char>& visited_handle,
    const double stop_energy,
    const double current_max_energy,
    const double initial_target_edge_length)
{
    // two ring version
    if (m.top_simplex_type() != PrimitiveType::Tetrahedron) return;

    const auto coordinate_accessor = m.create_const_accessor<double>(coordinate_handle);
    const auto energy_accessor = m.create_const_accessor<double>(energy_handle);

    auto energy_filter_accessor = m.create_accessor<char>(energy_filter_handle);
    auto visited_accessor = m.create_accessor<char>(visited_handle);

    const double stop_filter_energy = stop_energy * 0.8;
    double filter_energy = std::max(current_max_energy / 100, stop_filter_energy);
    filter_energy = std::min(filter_energy, 100.0);

    auto vertices_all = m.get_all(PrimitiveType::Vertex);

    for (const auto& v : vertices_all) { // reset visited flag
        visited_accessor.scalar_attribute(v) = char(0);
        energy_filter_accessor.scalar_attribute(v) = char(0);
    }

    // get centroids and initial v_queue
    for (const auto& t : m.get_all(PrimitiveType::Tetrahedron)) {
        if (energy_accessor.const_scalar_attribute(t) < filter_energy) {
            // skip good tets
            continue;
        }
        auto vertices = m.orient_vertices(t);
        for (const auto& v : vertices) {
            energy_filter_accessor.scalar_attribute(v) = char(1);
            for (const auto& vv : simplex::k_ring(m, simplex::Simplex::vertex(m, v), 1)
                                      .simplex_vector(PrimitiveType::Vertex)) {
                energy_filter_accessor.scalar_attribute(vv) = char(1);
            }
        }
    }
}

void set_operation_energy_filter_after_sizing_field(
    Mesh& m,
    const TypedAttributeHandle<double>& coordinate_handle,
    const TypedAttributeHandle<double>& energy_handle,
    const TypedAttributeHandle<char>& energy_filter_handle,
    const TypedAttributeHandle<char>& visited_handle,
    const double stop_energy,
    const double current_max_energy,
    const double initial_target_edge_length)
{
    if (m.top_simplex_type() != PrimitiveType::Tetrahedron) return;

    const auto coordinate_accessor = m.create_const_accessor<double>(coordinate_handle);
    const auto energy_accessor = m.create_const_accessor<double>(energy_handle);

    auto energy_filter_accessor = m.create_accessor<char>(energy_filter_handle);
    auto visited_accessor = m.create_accessor<char>(visited_handle);

    const double stop_filter_energy = stop_energy * 0.8;
    double filter_energy = std::max(current_max_energy / 100, stop_filter_energy);
    filter_energy = std::min(filter_energy, 100.0);

    auto vertices_all = m.get_all(PrimitiveType::Vertex);
    std::vector<Vector3d> centroids;
    std::queue<Tuple> v_queue;

    // get centroids and initial v_queue
    for (const auto& t : m.get_all(PrimitiveType::Tetrahedron)) {
        if (energy_accessor.const_scalar_attribute(t) < filter_energy) {
            // skip good tets
            continue;
        }
        auto vertices = m.orient_vertices(t);
        Vector3d c(0, 0, 0);
        for (int i = 0; i < 4; ++i) {
            c += coordinate_accessor.const_vector_attribute(vertices[i]);
            v_queue.emplace(vertices[i]);
        }
        centroids.emplace_back(c / 4.0);
    }

    const double R = initial_target_edge_length * 1.8;

    for (const auto& v : vertices_all) { // reset visited flag
        visited_accessor.scalar_attribute(v) = char(0);
        energy_filter_accessor.scalar_attribute(v) = char(0);
    }

    // TODO: use efficient data structure
    auto get_nearest_dist = [&](const Tuple& v) -> double {
        Tuple nearest_tuple;
        double min_dist = std::numeric_limits<double>::max();
        const Vector3d v_pos = coordinate_accessor.const_vector_attribute(v);
        for (const auto& c_pos : centroids) {
            double dist = (c_pos - v_pos).norm();
            min_dist = std::min(min_dist, dist);
        }
        return min_dist;
    };

    while (!v_queue.empty()) {
        auto v = v_queue.front();
        v_queue.pop();

        if (visited_accessor.scalar_attribute(v) == char(1)) continue;
        visited_accessor.scalar_attribute(v) = char(1);

        double dist = std::max(0., get_nearest_dist(v));

        if (dist > R) {
            visited_accessor.scalar_attribute(v) = char(0);
            continue;
        }

        energy_filter_accessor.scalar_attribute(v) = char(1);

        // push one ring vertices into the queue
        for (const auto& v_one_ring : simplex::link(m, simplex::Simplex::vertex(m, v))
                                          .simplex_vector(PrimitiveType::Vertex)) {
            if (visited_accessor.scalar_attribute(v_one_ring) == char(1)) continue;
            v_queue.push(v_one_ring.tuple());
        }
    }
}

void wildmeshing(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    // opt_logger().set_level(spdlog::level::info);

    //////////////////////////////////
    // Load mesh from settings
    WildmeshingOptions options = j.get<WildmeshingOptions>();
    const std::filesystem::path& file = options.input;
    auto mesh = cache.read_mesh(options.input);

    if (!mesh->is_connectivity_valid()) {
        throw std::runtime_error("input mesh for wildmeshing connectivity invalid");
    }

    //

    //////////////////////////////////
    // Retriving vertices
    attribute::MeshAttributeHandle pt_attribute =
        mesh->get_attribute_handle<double>(options.attributes.position, PrimitiveType::Vertex);
    auto pt_accessor = mesh->create_accessor(pt_attribute.as<double>());


    //////////////////////////////////
    // computing bbox diagonal
    Eigen::VectorXd bmin(mesh->top_cell_dimension());
    bmin.setConstant(std::numeric_limits<double>::max());
    Eigen::VectorXd bmax(mesh->top_cell_dimension());
    bmax.setConstant(std::numeric_limits<double>::lowest());

    const auto vertices = mesh->get_all(PrimitiveType::Vertex);
    for (const auto& v : vertices) {
        const auto p = pt_accessor.vector_attribute(v);
        for (int64_t d = 0; d < bmax.size(); ++d) {
            bmin[d] = std::min(bmin[d], p[d]);
            bmax[d] = std::max(bmax[d], p[d]);
        }
    }


    const double bbdiag = (bmax - bmin).norm();

    wmtk::logger().info("bbox max {}, bbox min {}, diag {}", bmax, bmin, bbdiag);

    const double target_edge_length = options.target_edge_length * bbdiag;

    // const double target_edge_length =
    //     options.target_edge_length * bbdiag /
    //     std::min(
    //         2.0,
    //         1 + options.target_edge_length * 2); // min to prevent bad option.target_edge_length

    wmtk::logger().info("target edge length: {}", target_edge_length);

    //////////////////////////////////
    // store amips
    auto amips_attribute =
        mesh->register_attribute<double>("wildmeshing_amips", mesh->top_simplex_type(), 1);
    auto amips_accessor = mesh->create_accessor(amips_attribute.as<double>());
    // amips update
    auto compute_amips = [](const Eigen::MatrixX<double>& P) -> Eigen::VectorXd {
        assert(P.rows() == 2 || P.rows() == 3); // rows --> attribute dimension
        assert(P.cols() == P.rows() + 1);
        if (P.cols() == 3) {
            // triangle
            assert(P.rows() == 2);
            std::array<double, 6> pts;
            for (size_t i = 0; i < 3; ++i) {
                for (size_t j = 0; j < 2; ++j) {
                    pts[2 * i + j] = P(j, i);
                }
            }
            const double a = Tri_AMIPS_energy(pts);
            return Eigen::VectorXd::Constant(1, a);
        } else {
            // tet
            assert(P.rows() == 3);
            std::array<double, 12> pts;
            for (size_t i = 0; i < 4; ++i) {
                for (size_t j = 0; j < 3; ++j) {
                    pts[3 * i + j] = P(j, i);
                }
            }
            const double a = Tet_AMIPS_energy(pts);
            return Eigen::VectorXd::Constant(1, a);
        }
    };
    auto amips_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            amips_attribute,
            pt_attribute,
            compute_amips);
    amips_update->run_on_all();

    double max_amips = std::numeric_limits<double>::lowest();
    double min_amips = std::numeric_limits<double>::max();

    for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
        // double e = amips->get_value(simplex::Simplex(mesh->top_simplex_type(), t));
        double e = amips_accessor.scalar_attribute(t);
        max_amips = std::max(max_amips, e);
        min_amips = std::min(min_amips, e);
    }

    logger().info("Initial Max AMIPS Energy: {}, Min AMIPS Energy: {}", max_amips, min_amips);

    //////////////////////////////////
    // sizing field scalar
    //////////////////////////////////

    auto sizing_field_scalar_attribute = mesh->register_attribute<double>(
        "sizing_field_scalar",
        PrimitiveType::Vertex,
        1,
        false,
        1); // defaults to 1


    //////////////////////////////////
    // Storing target edge length
    auto target_edge_length_attribute = mesh->register_attribute<double>(
        "wildmeshing_target_edge_length",
        PrimitiveType::Edge,
        1,
        false,
        target_edge_length); // defaults to target edge length

    // Target edge length update
    const double min_edge_length =
        options.envelopes.empty()
            ? 1e-6 // some default value if no envelope exists
            : options.envelopes[0].thickness; // use envelope thickness if available
    const double target_max_amips = options.target_max_amips;

    // Target Edge length update
    // auto compute_target_edge_length = [&](const Eigen::Vector2d& P) -> double {
    //     return (P[0] + P[1]) / 2 * target_edge_length;
    // };
    // auto target_edge_length_update =
    //     std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
    //         target_edge_length_attribute,
    //         sizing_field_scalar_attribute,
    //         compute_target_edge_length);

    // auto compute_target_edge_length =
    //     [target_edge_length,
    //      target_max_amips,
    //      min_edge_length,
    //      target_edge_length_attribute,
    //      &mesh](const Eigen::MatrixXd& P, const std::vector<Tuple>& neighs) ->
    //      Eigen::VectorXd {
    //     auto target_edge_length_accessor =
    //         mesh->create_accessor(target_edge_length_attribute.as<double>());

    //     assert(P.rows() == 1); // rows --> attribute dimension
    //     assert(!neighs.empty());
    //     assert(P.cols() == neighs.size());
    //     const double current_target_edge_length =
    //         target_edge_length_accessor.const_scalar_attribute(neighs[0]);
    //     const double max_amips = P.maxCoeff();

    //     double new_target_edge_length = current_target_edge_length;
    //     if (max_amips > target_max_amips) {
    //         new_target_edge_length *= 0.5;
    //     } else {
    //         new_target_edge_length *= 1.5;
    //     }
    //     new_target_edge_length =
    //         std::min(new_target_edge_length, target_edge_length); // upper bound
    //     new_target_edge_length = std::max(new_target_edge_length, min_edge_length); // lower bound

    //     return Eigen::VectorXd::Constant(1, new_target_edge_length);
    // };
    // auto target_edge_length_update =
    //     std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
    //         target_edge_length_attribute,
    //         amips_attribute,
    //         compute_target_edge_length);


    //// Example for some other target edge length
    // auto compute_target_edge_length = [](const Eigen::MatrixX<double>& P) ->
    // Eigen::VectorXd {
    //    assert(P.cols() == 2); // cols --> number of neighbors
    //    assert(P.rows() == 2 || P.rows() == 3); // rows --> attribute dimension
    //    const double x_avg = 0.5 * (P(0, 0) + P(0, 1));
    //    const double target_length = x_avg * x_avg;
    //    return Eigen::VectorXd::Constant(1, target_length);
    //};
    // auto target_edge_length_update =
    //    std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
    //        target_edge_length_attribute,
    //        rpt_attribute,
    //        compute_target_edge_length);
    // target_edge_length_update->run_on_all();


    //////////////////////////////////
    // Storing edge lengths
    auto edge_length_attribute =
        mesh->register_attribute<double>("edge_length", PrimitiveType::Edge, 1);
    auto edge_length_accessor = mesh->create_accessor(edge_length_attribute.as<double>());
    // Edge length update
    auto compute_edge_length = [](const Eigen::MatrixX<double>& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorXd::Constant(1, sqrt((P.col(0) - P.col(1)).squaredNorm()));
    };
    auto edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            edge_length_attribute,
            pt_attribute,
            compute_edge_length);
    edge_length_update->run_on_all();

    //////////////////////////////////
    // default transfer
    auto pass_through_attributes = base::get_attributes(cache, *mesh, options.pass_through);
    pass_through_attributes.push_back(edge_length_attribute);
    pass_through_attributes.push_back(amips_attribute);
    // pass_through_attributes.push_back(target_edge_length_attribute);


    //////////////////////////////////
    // Lambdas for priority
    //////////////////////////////////
    auto long_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return -edge_length_accessor.scalar_attribute(s.tuple());
    };
    auto short_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return edge_length_accessor.scalar_attribute(s.tuple());
    };


    //////////////////////////////////
    // envelopes
    //////////////////////////////////

    auto propagate_to_child_position =
        [](const Eigen::MatrixX<double>& P) -> Eigen::VectorX<double> { return P; };

    auto propagate_to_parent_position =
        [](const Eigen::MatrixX<double>& P) -> Eigen::VectorX<double> {
        assert(P.cols() == 1);
        return P.col(0);
    };
    using MeshConstrainPair = ProjectOperation::MeshConstrainPair;

    auto envelope_invariant = std::make_shared<InvariantCollection>(*mesh);
    std::vector<std::shared_ptr<SingleAttributeTransferStrategy<double, double>>>
        update_child_positon, update_parent_positon;
    std::vector<std::shared_ptr<Mesh>> envelopes;
    std::vector<MeshConstrainPair> mesh_constraint_pairs;

    std::vector<std::shared_ptr<Mesh>> multimesh_meshes;

    assert(options.envelopes.size() == 4); // four kind of envelopes in tetwild [surface_mesh,
                                           // open_boudnary, nonmanifold_edges, is_boundary(bbox)]
    // TODO: add nonmanifold vertex point mesh

    for (const auto& v : options.envelopes) {
        auto envelope = cache.read_mesh(v.geometry.mesh);
        if (envelope == nullptr) {
            wmtk::logger().info("TetWild: no {} mesh for this mesh", v.geometry.mesh);
            continue;
        } else {
            wmtk::logger().info("TetWild: registered {} mesh", v.geometry.mesh);
        }
        envelopes.emplace_back(envelope);

        auto constrained = base::get_attributes(cache, *mesh, v.constrained_position);
        multimesh_meshes.push_back(constrained.front().mesh().shared_from_this());
        assert(constrained.size() == 1);
        pass_through_attributes.emplace_back(constrained.front());

        auto envelope_position_handle =
            v.geometry.mesh == "input"
                ? envelope->get_attribute_handle<double>(v.geometry.position, PrimitiveType::Vertex)
                : envelope->get_attribute_handle<double>(
                      v.geometry.position,
                      PrimitiveType::Vertex);
        // envelope->get_attribute_handle<double>(v.geometry.position, PrimitiveType::Vertex);


        mesh_constraint_pairs.emplace_back(envelope_position_handle, constrained.front());

        envelope_invariant->add(std::make_shared<EnvelopeInvariant>(
            envelope_position_handle,
            v.thickness * bbdiag,
            constrained.front()));

        update_parent_positon.emplace_back(
            std::make_shared<SingleAttributeTransferStrategy<double, double>>(
                pt_attribute,
                constrained.front(),
                propagate_to_parent_position));

        update_child_positon.emplace_back(
            std::make_shared<SingleAttributeTransferStrategy<double, double>>(
                constrained.front(),
                pt_attribute,
                propagate_to_child_position));
    }

    //////////////////////////////////


    //////////////////////////////////
    // Invariants
    //////////////////////////////////

    auto inversion_invariant =
        std::make_shared<SimplexInversionInvariant<double>>(*mesh, pt_attribute.as<double>());

    std::shared_ptr<function::PerSimplexFunction> amips =
        std::make_shared<AMIPS>(*mesh, pt_attribute);
    // auto function_invariant = std::make_shared<FunctionInvariant>(mesh->top_simplex_type(),
    // amips);
    auto function_invariant =
        std::make_shared<MaxFunctionInvariant>(mesh->top_simplex_type(), amips);

    auto link_condition = std::make_shared<MultiMeshLinkConditionInvariant>(*mesh);

    auto todo_larger = std::make_shared<TodoLargerInvariant>(
        *mesh,
        edge_length_attribute.as<double>(),
        target_edge_length_attribute.as<double>(),
        4.0 / 3.0);

    auto todo_smaller = std::make_shared<TodoSmallerInvariant>(
        *mesh,
        edge_length_attribute.as<double>(),
        target_edge_length_attribute.as<double>(),
        4.0 / 5.0);


    auto interior_edge = std::make_shared<InteriorEdgeInvariant>(*mesh);
    auto interior_face = std::make_shared<InteriorSimplexInvariant>(*mesh, PrimitiveType::Triangle);

    for (const auto& em : multimesh_meshes) {
        interior_edge->add_boundary(*em);
        interior_face->add_boundary(*em);
    }

    auto valence_3 = std::make_shared<EdgeValenceInvariant>(*mesh, 3);
    auto valence_4 = std::make_shared<EdgeValenceInvariant>(*mesh, 4);
    auto swap44_energy_before =
        std::make_shared<Swap44EnergyBeforeInvariantDouble>(*mesh, pt_attribute.as<double>(), 0);
    auto swap44_2_energy_before =
        std::make_shared<Swap44EnergyBeforeInvariantDouble>(*mesh, pt_attribute.as<double>(), 1);
    auto swap32_energy_before =
        std::make_shared<Swap32EnergyBeforeInvariantDouble>(*mesh, pt_attribute.as<double>());
    auto swap23_energy_before =
        std::make_shared<Swap23EnergyBeforeInvariantDouble>(*mesh, pt_attribute.as<double>());

    auto invariant_separate_substructures =
        std::make_shared<invariants::SeparateSubstructuresInvariant>(*mesh);

    //////////////////////////////////
    // renew flags
    //////////////////////////////////
    auto visited_edge_flag =
        mesh->register_attribute<char>("visited_edge", PrimitiveType::Edge, 1, false, char(1));

    auto update_flag_func = [](const Eigen::MatrixX<double>& P) -> Eigen::VectorX<char> {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorX<char>::Constant(1, char(1));
    };
    auto tag_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<char, double>>(
            visited_edge_flag,
            pt_attribute,
            update_flag_func);

    //////////////////////////////////
    // sizing field update flags
    //////////////////////////////////
    auto visited_vertex_flag =
        mesh->register_attribute<char>("visited_vertex", PrimitiveType::Vertex, 1, false, char(1));
    pass_through_attributes.push_back(visited_vertex_flag);

    //////////////////////////////////
    // energy filter flag
    //////////////////////////////////
    auto energy_filter_attribute =
        mesh->register_attribute<char>("energy_filter", PrimitiveType::Vertex, 1, false, char(1));

    auto energy_filter_accessor = mesh->create_accessor<char>(energy_filter_attribute);

    auto update_energy_filter_func = [](const Eigen::MatrixX<double>& P) -> Eigen::VectorX<char> {
        // assert(P.cols() == 2);
        // assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorX<char>::Constant(1, char(1));
    };
    auto energy_filter_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<char, double>>(
            energy_filter_attribute,
            pt_attribute,
            update_energy_filter_func);

    pass_through_attributes.push_back(energy_filter_attribute);

    //////////////////////////////////
    // coloring flag for smoothing
    //////////////////////////////////

    // auto coloring_attribute =
    //     mesh->register_attribute<int64_t>("coloring", PrimitiveType::Vertex, 1, false, -1);
    // pass_through_attributes.push_back(coloring_attribute);

    //////////////////////////////////
    // Creation of the 4 ops
    //////////////////////////////////
    std::vector<std::shared_ptr<Operation>> ops;
    std::vector<std::string> ops_name;

    //////////////////////////////////
    // 1) EdgeSplit
    //////////////////////////////////
    auto split = std::make_shared<EdgeSplit>(*mesh);
    split->set_priority(long_edges_first);

    split->add_invariant(todo_larger);
    split->add_invariant(inversion_invariant);

    split->set_new_attribute_strategy(pt_attribute);
    split->set_new_attribute_strategy(sizing_field_scalar_attribute);
    split->set_new_attribute_strategy(
        visited_edge_flag,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    split->set_new_attribute_strategy(
        target_edge_length_attribute,
        wmtk::operations::SplitBasicStrategy::Copy,
        wmtk::operations::SplitRibBasicStrategy::Mean);

    for (const auto& attr : pass_through_attributes) {
        split->set_new_attribute_strategy(
            attr,
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::None);
    }

    split->add_transfer_strategy(amips_update);
    split->add_transfer_strategy(edge_length_update);
    split->add_transfer_strategy(tag_update); // for renew the queue
    split->add_transfer_strategy(energy_filter_update);

    // split->add_transfer_strategy(target_edge_length_update);

    auto split_then_round = std::make_shared<AndOperationSequence>(*mesh);
    split_then_round->add_operation(split);

    for (auto& s : update_child_positon) {
        split_then_round->add_transfer_strategy(s);
    }

    // split unrounded
    auto split_unrounded = std::make_shared<EdgeSplit>(*mesh);
    split_unrounded->set_priority(long_edges_first);

    split_unrounded->add_invariant(todo_larger);

    auto split_unrounded_transfer_strategy =
        std::make_shared<SplitNewAttributeStrategy<double>>(pt_attribute);
    split_unrounded_transfer_strategy->set_strategy(
        [](const Eigen::VectorX<double>& a, const std::bitset<2>&) {
            return std::array<Eigen::VectorX<double>, 2>{{a, a}};
        });
    split_unrounded_transfer_strategy->set_rib_strategy(
        [](const Eigen::VectorX<double>& p0_d,
           const Eigen::VectorX<double>& p1_d,
           const std::bitset<2>& bs) -> Eigen::VectorX<double> {
            Eigen::VectorX<double> p0(p0_d.size());
            Eigen::VectorX<double> p1(p1_d.size());
            for (int i = 0; i < p0_d.size(); ++i) {
                p0[i] = p0_d[i];
                p1[i] = p1_d[i];
            }
            if (bs[0] == bs[1]) {
                return (p0 + p1) / 2.0;
            } else if (bs[0]) {
                return p0;

            } else {
                return p1;
            }
        });

    split_unrounded->set_new_attribute_strategy(pt_attribute, split_unrounded_transfer_strategy);
    split_unrounded->set_new_attribute_strategy(sizing_field_scalar_attribute);
    split_unrounded->set_new_attribute_strategy(
        visited_edge_flag,
        wmtk::operations::SplitBasicStrategy::None,
        wmtk::operations::SplitRibBasicStrategy::None);
    for (const auto& attr : pass_through_attributes) {
        split_unrounded->set_new_attribute_strategy(
            attr,
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::None);
    }
    split_unrounded->set_new_attribute_strategy(
        target_edge_length_attribute,
        wmtk::operations::SplitBasicStrategy::Copy,
        wmtk::operations::SplitRibBasicStrategy::Mean);

    split_unrounded->add_transfer_strategy(amips_update);
    split_unrounded->add_transfer_strategy(edge_length_update);
    split_unrounded->add_transfer_strategy(tag_update); // for renew the queue
    split_unrounded->add_transfer_strategy(energy_filter_update);

    // split->add_transfer_strategy(target_edge_length_update);

    auto split_sequence = std::make_shared<OrOperationSequence>(*mesh);
    split_sequence->add_operation(split_then_round);
    split_sequence->add_operation(split_unrounded);
    split_sequence->add_invariant(
        std::make_shared<EnergyFilterInvariant>(*mesh, energy_filter_attribute.as<char>()));

    split_sequence->set_priority(long_edges_first);

    ops.emplace_back(split_sequence);
    ops_name.emplace_back("SPLIT");


    //////////////////////////////////
    // collapse transfer
    //////////////////////////////////
    auto clps_strat1 = std::make_shared<CollapseNewAttributeStrategy<double>>(pt_attribute);
    // clps_strat1->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
    // clps_strat1->set_strategy(CollapseBasicStrategy::Default);
    clps_strat1->set_strategy(CollapseBasicStrategy::CopyOther);

    auto clps_strat2 = std::make_shared<CollapseNewAttributeStrategy<double>>(pt_attribute);
    // clps_strat2->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
    // clps_strat2->set_strategy(CollapseBasicStrategy::Default);
    clps_strat2->set_strategy(CollapseBasicStrategy::CopyTuple);

    //////////////////////////////////
    // 2) EdgeCollapse
    //////////////////////////////////

    auto setup_collapse = [&](std::shared_ptr<EdgeCollapse>& collapse) {
        collapse->add_invariant(invariant_separate_substructures);
        collapse->add_invariant(link_condition);
        collapse->add_invariant(inversion_invariant);
        // collapse->add_invariant(function_invariant);
        collapse->add_invariant(envelope_invariant);

        collapse->set_new_attribute_strategy(
            visited_edge_flag,
            wmtk::operations::CollapseBasicStrategy::None);

        collapse->add_transfer_strategy(tag_update);
        collapse->add_transfer_strategy(energy_filter_update);
        for (const auto& attr : pass_through_attributes) {
            collapse->set_new_attribute_strategy(
                attr,
                wmtk::operations::CollapseBasicStrategy::None);
        }
        collapse->set_new_attribute_strategy(
            target_edge_length_attribute,
            wmtk::operations::CollapseBasicStrategy::None);
        // THis triggers a segfault in release
        // solved somehow
        // collapse->set_priority(short_edges_first);

        collapse->add_transfer_strategy(amips_update);
        collapse->add_transfer_strategy(edge_length_update);
        // collapse->add_transfer_strategy(target_edge_length_update);

        for (auto& s : update_child_positon) {
            collapse->add_transfer_strategy(s);
        }
    };

    auto collapse1 = std::make_shared<EdgeCollapse>(*mesh);
    collapse1->add_invariant(std::make_shared<CollapseEnergyBeforeInvariantDouble>(
        *mesh,
        pt_attribute.as<double>(),
        amips_attribute.as<double>(),
        1));

    collapse1->set_new_attribute_strategy(pt_attribute, clps_strat1);
    collapse1->set_new_attribute_strategy(sizing_field_scalar_attribute, clps_strat1);
    setup_collapse(collapse1);

    auto collapse2 = std::make_shared<EdgeCollapse>(*mesh);
    collapse2->add_invariant(std::make_shared<CollapseEnergyBeforeInvariantDouble>(
        *mesh,
        pt_attribute.as<double>(),
        amips_attribute.as<double>(),
        0));

    collapse2->set_new_attribute_strategy(pt_attribute, clps_strat2);
    collapse2->set_new_attribute_strategy(sizing_field_scalar_attribute, clps_strat2);
    setup_collapse(collapse2);

    auto collapse = std::make_shared<OrOperationSequence>(*mesh);
    collapse->add_operation(collapse1);
    collapse->add_operation(collapse2);
    collapse->add_invariant(todo_smaller);

    auto collapse_then_round = std::make_shared<AndOperationSequence>(*mesh);
    collapse_then_round->add_operation(collapse);

    collapse_then_round->set_priority(short_edges_first);
    collapse_then_round->add_invariant(
        std::make_shared<EnergyFilterInvariant>(*mesh, energy_filter_attribute.as<char>()));


    for (auto& s : update_child_positon) {
        collapse_then_round->add_transfer_strategy(s);
    }

    ops.emplace_back(collapse_then_round);
    ops_name.emplace_back("COLLAPSE");

    //////////////////////////////////
    // 3) Swap
    //////////////////////////////////

    auto setup_swap = [&](Operation& op,
                          EdgeCollapse& collapse,
                          EdgeSplit& split,
                          std::shared_ptr<Invariant> simplex_invariant,
                          bool is_edge = true) {
        if (is_edge) op.set_priority(long_edges_first);

        op.add_invariant(simplex_invariant);
        op.add_invariant(inversion_invariant);
        // op.add_invariant(function_invariant);

        op.add_transfer_strategy(amips_update);
        op.add_transfer_strategy(edge_length_update);
        // op.add_transfer_strategy(target_edge_length_update);
        // for (auto& s : update_child_positon) {
        //     op.add_transfer_strategy(s);
        // }

        collapse.add_invariant(invariant_separate_substructures);
        collapse.add_invariant(link_condition);


        collapse.set_new_attribute_strategy(pt_attribute, CollapseBasicStrategy::CopyOther);
        split.set_new_attribute_strategy(pt_attribute);

        split.set_new_attribute_strategy(
            target_edge_length_attribute,
            wmtk::operations::SplitBasicStrategy::Copy,
            wmtk::operations::SplitRibBasicStrategy::Mean);
        collapse.set_new_attribute_strategy(
            target_edge_length_attribute,
            wmtk::operations::CollapseBasicStrategy::None);

        // this might not be necessary
        // for (auto& s : update_child_positon) {
        //     collapse.add_transfer_strategy(s);
        //     split.add_transfer_strategy(s);
        // }


        for (const auto& attr : pass_through_attributes) {
            split.set_new_attribute_strategy(
                attr,
                wmtk::operations::SplitBasicStrategy::None,
                wmtk::operations::SplitRibBasicStrategy::None);
            collapse.set_new_attribute_strategy(
                attr,
                wmtk::operations::CollapseBasicStrategy::None);
        }
    };


    if (mesh->top_simplex_type() == PrimitiveType::Triangle) {
        auto swap = std::make_shared<TriEdgeSwap>(*mesh);
        setup_swap(*swap, swap->collapse(), swap->split(), interior_edge);

        ops.push_back(swap);
        ops_name.push_back("swap");

    } else if (mesh->top_simplex_type() == PrimitiveType::Tetrahedron) {
        auto swap56 = std::make_shared<MinOperationSequence>(*mesh);
        for (int i = 0; i < 5; ++i) {
            auto swap = std::make_shared<TetEdgeSwap>(*mesh, i);
            swap->collapse().add_invariant(invariant_separate_substructures);
            swap->collapse().add_invariant(link_condition);
            swap->collapse().set_new_attribute_strategy(
                pt_attribute,
                CollapseBasicStrategy::CopyOther);
            swap->collapse().set_new_attribute_strategy(
                sizing_field_scalar_attribute,
                CollapseBasicStrategy::CopyOther);
            swap->split().set_new_attribute_strategy(pt_attribute);
            swap->split().set_new_attribute_strategy(sizing_field_scalar_attribute);
            // swap->split().add_transfer_strategy(amips_update);
            // swap->collapse().add_transfer_strategy(amips_update);
            swap->split().set_new_attribute_strategy(
                visited_edge_flag,
                wmtk::operations::SplitBasicStrategy::None,
                wmtk::operations::SplitRibBasicStrategy::None);
            swap->collapse().set_new_attribute_strategy(
                visited_edge_flag,
                wmtk::operations::CollapseBasicStrategy::None);

            swap->split().set_new_attribute_strategy(
                target_edge_length_attribute,
                wmtk::operations::SplitBasicStrategy::Copy,
                wmtk::operations::SplitRibBasicStrategy::Mean);
            swap->collapse().set_new_attribute_strategy(
                target_edge_length_attribute,
                wmtk::operations::CollapseBasicStrategy::None);

            swap->add_invariant(std::make_shared<Swap56EnergyBeforeInvariantDouble>(
                *mesh,
                pt_attribute.as<double>(),
                i));

            swap->add_transfer_strategy(amips_update);

            // swap->add_invariant(inversion_invariant);
            swap->collapse().add_invariant(inversion_invariant);

            // swap->collapse().add_invariant(envelope_invariant);

            for (const auto& attr : pass_through_attributes) {
                swap->split().set_new_attribute_strategy(
                    attr,
                    wmtk::operations::SplitBasicStrategy::None,
                    wmtk::operations::SplitRibBasicStrategy::None);
                swap->collapse().set_new_attribute_strategy(
                    attr,
                    wmtk::operations::CollapseBasicStrategy::None);
            }

            swap56->add_operation(swap);
        }

        auto swap56_energy_check = [&](int64_t idx, const simplex::Simplex& t) -> double {
            constexpr static PrimitiveType PV = PrimitiveType::Vertex;
            constexpr static PrimitiveType PE = PrimitiveType::Edge;
            constexpr static PrimitiveType PF = PrimitiveType::Triangle;
            constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

            auto accessor = mesh->create_const_accessor(pt_attribute.as<double>());

            const Tuple e0 = t.tuple();
            const Tuple e1 = mesh->switch_tuple(e0, PV);

            std::array<Tuple, 5> v;
            auto iter_tuple = e0;
            for (int64_t i = 0; i < 5; ++i) {
                v[i] = mesh->switch_tuples(iter_tuple, {PE, PV});
                iter_tuple = mesh->switch_tuples(iter_tuple, {PF, PT});
            }
            if (iter_tuple != e0) return 0;
            assert(iter_tuple == e0);

            // five iterable vertices remap to 0-4 by m_collapse_index, 0: m_collapse_index, 5:
            // e0, 6: e1
            std::array<Eigen::Vector3<double>, 7> positions = {
                {accessor.const_vector_attribute(v[(idx + 0) % 5]),
                 accessor.const_vector_attribute(v[(idx + 1) % 5]),
                 accessor.const_vector_attribute(v[(idx + 2) % 5]),
                 accessor.const_vector_attribute(v[(idx + 3) % 5]),
                 accessor.const_vector_attribute(v[(idx + 4) % 5]),
                 accessor.const_vector_attribute(e0),
                 accessor.const_vector_attribute(e1)}};

            std::array<Eigen::Vector3d, 7> positions_double = {
                {positions[0],
                 positions[1],
                 positions[2],
                 positions[3],
                 positions[4],
                 positions[5],
                 positions[6]}};

            std::array<std::array<int, 4>, 6> new_tets = {
                {{{0, 1, 2, 5}},
                 {{0, 2, 3, 5}},
                 {{0, 3, 4, 5}},
                 {{0, 1, 2, 6}},
                 {{0, 2, 3, 6}},
                 {{0, 3, 4, 6}}}};

            double new_energy_max = std::numeric_limits<double>::lowest();

            for (int i = 0; i < 6; ++i) {
                if (wmtk::utils::wmtk_orient3d(
                        positions[new_tets[i][0]],
                        positions[new_tets[i][1]],
                        positions[new_tets[i][2]],
                        positions[new_tets[i][3]]) > 0) {
                    auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                        positions_double[new_tets[i][0]][0],
                        positions_double[new_tets[i][0]][1],
                        positions_double[new_tets[i][0]][2],
                        positions_double[new_tets[i][1]][0],
                        positions_double[new_tets[i][1]][1],
                        positions_double[new_tets[i][1]][2],
                        positions_double[new_tets[i][2]][0],
                        positions_double[new_tets[i][2]][1],
                        positions_double[new_tets[i][2]][2],
                        positions_double[new_tets[i][3]][0],
                        positions_double[new_tets[i][3]][1],
                        positions_double[new_tets[i][3]][2],
                    }});


                    if (energy > new_energy_max) new_energy_max = energy;
                } else {
                    auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                        positions_double[new_tets[i][1]][0],
                        positions_double[new_tets[i][1]][1],
                        positions_double[new_tets[i][1]][2],
                        positions_double[new_tets[i][0]][0],
                        positions_double[new_tets[i][0]][1],
                        positions_double[new_tets[i][0]][2],
                        positions_double[new_tets[i][2]][0],
                        positions_double[new_tets[i][2]][1],
                        positions_double[new_tets[i][2]][2],
                        positions_double[new_tets[i][3]][0],
                        positions_double[new_tets[i][3]][1],
                        positions_double[new_tets[i][3]][2],
                    }});


                    if (energy > new_energy_max) new_energy_max = energy;
                }
            }

            return new_energy_max;
        };

        swap56->set_value_function(swap56_energy_check);
        swap56->add_invariant(std::make_shared<EdgeValenceInvariant>(*mesh, 5));

        // swap44

        auto swap44 = std::make_shared<MinOperationSequence>(*mesh);
        for (int i = 0; i < 2; ++i) {
            auto swap = std::make_shared<TetEdgeSwap>(*mesh, i);
            swap->collapse().add_invariant(invariant_separate_substructures);
            swap->collapse().add_invariant(link_condition);
            swap->collapse().set_new_attribute_strategy(
                pt_attribute,
                CollapseBasicStrategy::CopyOther);
            swap->collapse().set_new_attribute_strategy(
                sizing_field_scalar_attribute,
                CollapseBasicStrategy::CopyOther);
            swap->split().set_new_attribute_strategy(pt_attribute);
            swap->split().set_new_attribute_strategy(sizing_field_scalar_attribute);
            swap->split().set_new_attribute_strategy(
                visited_edge_flag,
                wmtk::operations::SplitBasicStrategy::None,
                wmtk::operations::SplitRibBasicStrategy::None);
            swap->collapse().set_new_attribute_strategy(
                visited_edge_flag,
                wmtk::operations::CollapseBasicStrategy::None);

            // swap->split().add_transfer_strategy(amips_update);
            // swap->collapse().add_transfer_strategy(amips_update);

            swap->split().set_new_attribute_strategy(
                target_edge_length_attribute,
                wmtk::operations::SplitBasicStrategy::Copy,
                wmtk::operations::SplitRibBasicStrategy::Mean);
            swap->collapse().set_new_attribute_strategy(
                target_edge_length_attribute,
                wmtk::operations::CollapseBasicStrategy::None);

            swap->add_transfer_strategy(amips_update);

            swap->add_invariant(std::make_shared<Swap44EnergyBeforeInvariantDouble>(
                *mesh,
                pt_attribute.as<double>(),
                i));

            // swap->add_invariant(inversion_invariant);
            swap->collapse().add_invariant(inversion_invariant);

            // swap->collapse().add_invariant(envelope_invariant);

            for (const auto& attr : pass_through_attributes) {
                swap->split().set_new_attribute_strategy(
                    attr,
                    wmtk::operations::SplitBasicStrategy::None,
                    wmtk::operations::SplitRibBasicStrategy::None);
                swap->collapse().set_new_attribute_strategy(
                    attr,
                    wmtk::operations::CollapseBasicStrategy::None);
            }

            swap44->add_operation(swap);
        }

        auto swap44_energy_check = [&](int64_t idx, const simplex::Simplex& t) -> double {
            constexpr static PrimitiveType PV = PrimitiveType::Vertex;
            constexpr static PrimitiveType PE = PrimitiveType::Edge;
            constexpr static PrimitiveType PF = PrimitiveType::Triangle;
            constexpr static PrimitiveType PT = PrimitiveType::Tetrahedron;

            auto accessor = mesh->create_const_accessor(pt_attribute.as<double>());

            // get the coords of the vertices
            // input edge end points
            const Tuple e0 = t.tuple();
            const Tuple e1 = mesh->switch_tuple(e0, PV);
            // other four vertices
            std::array<Tuple, 4> v;
            auto iter_tuple = e0;
            for (int64_t i = 0; i < 4; ++i) {
                v[i] = mesh->switch_tuples(iter_tuple, {PE, PV});
                iter_tuple = mesh->switch_tuples(iter_tuple, {PF, PT});
            }

            if (iter_tuple != e0) return 0;
            assert(iter_tuple == e0);

            std::array<Eigen::Vector3<double>, 6> positions = {
                {accessor.const_vector_attribute(v[(idx + 0) % 4]),
                 accessor.const_vector_attribute(v[(idx + 1) % 4]),
                 accessor.const_vector_attribute(v[(idx + 2) % 4]),
                 accessor.const_vector_attribute(v[(idx + 3) % 4]),
                 accessor.const_vector_attribute(e0),
                 accessor.const_vector_attribute(e1)}};
            std::array<Eigen::Vector3d, 6> positions_double = {
                {positions[0],
                 positions[1],
                 positions[2],
                 positions[3],
                 positions[4],
                 positions[5]}};

            std::array<std::array<int, 4>, 4> new_tets = {
                {{{0, 1, 2, 4}}, {{0, 2, 3, 4}}, {{0, 1, 2, 5}}, {{0, 2, 3, 5}}}};

            double new_energy_max = std::numeric_limits<double>::lowest();

            for (int i = 0; i < 4; ++i) {
                if (wmtk::utils::wmtk_orient3d(
                        positions[new_tets[i][0]],
                        positions[new_tets[i][1]],
                        positions[new_tets[i][2]],
                        positions[new_tets[i][3]]) > 0) {
                    auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                        positions_double[new_tets[i][0]][0],
                        positions_double[new_tets[i][0]][1],
                        positions_double[new_tets[i][0]][2],
                        positions_double[new_tets[i][1]][0],
                        positions_double[new_tets[i][1]][1],
                        positions_double[new_tets[i][1]][2],
                        positions_double[new_tets[i][2]][0],
                        positions_double[new_tets[i][2]][1],
                        positions_double[new_tets[i][2]][2],
                        positions_double[new_tets[i][3]][0],
                        positions_double[new_tets[i][3]][1],
                        positions_double[new_tets[i][3]][2],
                    }});

                    if (energy > new_energy_max) new_energy_max = energy;
                } else {
                    auto energy = wmtk::function::utils::Tet_AMIPS_energy({{
                        positions_double[new_tets[i][1]][0],
                        positions_double[new_tets[i][1]][1],
                        positions_double[new_tets[i][1]][2],
                        positions_double[new_tets[i][0]][0],
                        positions_double[new_tets[i][0]][1],
                        positions_double[new_tets[i][0]][2],
                        positions_double[new_tets[i][2]][0],
                        positions_double[new_tets[i][2]][1],
                        positions_double[new_tets[i][2]][2],
                        positions_double[new_tets[i][3]][0],
                        positions_double[new_tets[i][3]][1],
                        positions_double[new_tets[i][3]][2],
                    }});

                    if (energy > new_energy_max) new_energy_max = energy;
                }
            }

            return new_energy_max;
        };

        swap44->set_value_function(swap44_energy_check);
        swap44->add_invariant(std::make_shared<EdgeValenceInvariant>(*mesh, 4));

        // swap 32
        auto swap32 = std::make_shared<TetEdgeSwap>(*mesh, 0);
        swap32->add_invariant(std::make_shared<EdgeValenceInvariant>(*mesh, 3));
        swap32->add_invariant(
            std::make_shared<Swap32EnergyBeforeInvariantDouble>(*mesh, pt_attribute.as<double>()));

        swap32->collapse().add_invariant(invariant_separate_substructures);
        swap32->collapse().add_invariant(link_condition);
        swap32->collapse().set_new_attribute_strategy(
            pt_attribute,
            CollapseBasicStrategy::CopyOther);
        swap32->collapse().set_new_attribute_strategy(
            sizing_field_scalar_attribute,
            CollapseBasicStrategy::CopyOther);
        // swap32->add_invariant(inversion_invariant);
        swap32->split().set_new_attribute_strategy(pt_attribute);
        swap32->split().set_new_attribute_strategy(sizing_field_scalar_attribute);
        swap32->split().set_new_attribute_strategy(
            visited_edge_flag,
            wmtk::operations::SplitBasicStrategy::None,
            wmtk::operations::SplitRibBasicStrategy::None);
        swap32->collapse().set_new_attribute_strategy(
            visited_edge_flag,
            wmtk::operations::CollapseBasicStrategy::None);

        // swap32->split().add_transfer_strategy(amips_update);
        // swap32->collapse().add_transfer_strategy(amips_update);

        swap32->split().set_new_attribute_strategy(
            target_edge_length_attribute,
            wmtk::operations::SplitBasicStrategy::Copy,
            wmtk::operations::SplitRibBasicStrategy::Mean);
        swap32->collapse().set_new_attribute_strategy(
            target_edge_length_attribute,
            wmtk::operations::CollapseBasicStrategy::None);

        swap32->add_transfer_strategy(amips_update);

        // hack
        swap32->collapse().add_invariant(inversion_invariant);
        // swap32->collapse().add_invariant(envelope_invariant);

        for (const auto& attr : pass_through_attributes) {
            swap32->split().set_new_attribute_strategy(
                attr,
                wmtk::operations::SplitBasicStrategy::None,
                wmtk::operations::SplitRibBasicStrategy::None);
            swap32->collapse().set_new_attribute_strategy(
                attr,
                wmtk::operations::CollapseBasicStrategy::None);
        }

        // all swaps

        auto swap_all = std::make_shared<OrOperationSequence>(*mesh);
        swap_all->add_operation(swap32);
        swap_all->add_operation(swap44);
        swap_all->add_operation(swap56);
        swap_all->add_transfer_strategy(tag_update);
        swap_all->add_transfer_strategy(energy_filter_update);

        auto swap_then_round = std::make_shared<AndOperationSequence>(*mesh);
        swap_then_round->add_operation(swap_all);
        swap_then_round->add_invariant(
            std::make_shared<EnergyFilterInvariant>(*mesh, energy_filter_attribute.as<char>()));
        swap_then_round->add_invariant(std::make_shared<InteriorEdgeInvariant>(*mesh));
        swap_then_round->add_invariant(std::make_shared<NoChildMeshAttachingInvariant>(*mesh));

        swap_then_round->set_priority(long_edges_first);


        // swap_then_round->add_invariant(inversion_invariant);
        for (auto& s : update_child_positon) {
            swap_then_round->add_transfer_strategy(s);
        }

        ops.push_back(swap_then_round);
        ops_name.push_back("EDGE SWAP");
    }

    // 4) Smoothing
    // //////////////////////////////////////
    // // special smoothing on surface
    // //////////////////////////////////////

    // if (mesh->top_simplex_type() == PrimitiveType::Tetrahedron) {
    //     for (auto& pair : mesh_constraint_pairs) {
    //         if (pair.second.mesh().top_simplex_type() != PrimitiveType::Triangle) continue;

    //         auto& child_mesh = pair.second.mesh();
    //         auto& child_position_handle = pair.second;

    //         auto lap_smoothing = std::make_shared<TetWildTangentialLaplacianSmoothing>(
    //             child_mesh,
    //             child_position_handle.as<double>());
    //         lap_smoothing->add_invariant(std::make_shared<RoundedInvariant>(
    //             child_mesh,
    //             child_position_handle.as<double>()));
    //         lap_smoothing->add_invariant(inversion_invariant);

    //         auto proj_lap_smoothing =
    //             std::make_shared<ProjectOperation>(lap_smoothing, mesh_constraint_pairs);
    //         proj_lap_smoothing->use_random_priority() = true;

    //         proj_lap_smoothing->add_invariant(envelope_invariant);
    //         proj_lap_smoothing->add_invariant(inversion_invariant);
    //         proj_lap_smoothing->add_invariant(
    //             std::make_shared<EnergyFilterInvariant>(*mesh,
    //             energy_filter_attribute.as<char>()));

    //         proj_lap_smoothing->add_transfer_strategy(amips_update);
    //         proj_lap_smoothing->add_transfer_strategy(edge_length_update);
    //         for (auto& s : update_parent_positon) { // TODO::this should from only one child
    //             proj_lap_smoothing->add_transfer_strategy(s);
    //         }
    //         for (auto& s : update_child_positon) {
    //             proj_lap_smoothing->add_transfer_strategy(s);
    //         }

    //         ops.push_back(proj_lap_smoothing);
    //         ops_name.push_back("LAPLACIAN SMOOTHING");
    //     }
    // }

    // auto energy =
    //     std::make_shared<function::LocalNeighborsSumFunction>(*mesh, pt_attribute, *amips);
    // auto smoothing = std::make_shared<OptimizationSmoothing>(energy);
    auto smoothing = std::make_shared<AMIPSOptimizationSmoothing>(*mesh, pt_attribute);
    smoothing->add_invariant(inversion_invariant);
    for (auto& s : update_child_positon) {
        smoothing->add_transfer_strategy(s);
    }

    // test code
    // smoothing->add_invariant(envelope_invariant);
    // smoothing->add_transfer_strategy(amips_update);
    // smoothing->add_transfer_strategy(edge_length_update);
    // ops.push_back(smoothing);
    // ops_name.push_back("SMOOTHING");
    //--------

    auto proj_smoothing = std::make_shared<ProjectOperation>(smoothing, mesh_constraint_pairs);
    proj_smoothing->use_random_priority() = true;

    proj_smoothing->add_invariant(envelope_invariant);
    proj_smoothing->add_invariant(inversion_invariant);
    proj_smoothing->add_invariant(
        std::make_shared<EnergyFilterInvariant>(*mesh, energy_filter_attribute.as<char>()));

    proj_smoothing->add_transfer_strategy(amips_update);
    proj_smoothing->add_transfer_strategy(edge_length_update);
    for (auto& s : update_parent_positon) {
        proj_smoothing->add_transfer_strategy(s);
    }

    for (auto& s : update_child_positon) {
        proj_smoothing->add_transfer_strategy(s);
    }
    // proj_smoothing->add_transfer_strategy(target_edge_length_update);

    for (int i = 0; i < 1; ++i) {
        ops.push_back(proj_smoothing);
        ops_name.push_back("SMOOTHING");
    }


    write(
        mesh,
        paths.output_dir,
        options.output,
        options.attributes.position,
        0,
        options.intermediate_output);

    auto surface_mesh = mesh->get_child_meshes().front();

    write(
        surface_mesh,
        paths.output_dir,
        options.output + "_intermediate_surface",
        options.attributes.position,
        0,
        options.intermediate_output);

    //////////////////////////////////
    // Running all ops in order n times
    Scheduler scheduler;
    int64_t success = 10;

    //////////////////////////////////
    // preprocessing
    //////////////////////////////////

    SchedulerStats pre_stats;

    logger().info("----------------------- Preprocess Collapse -----------------------");
    for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
        const auto vertices = mesh->orient_vertices(t);
        std::vector<Vector3d> pos;
        for (int i = 0; i < vertices.size(); ++i) {
            pos.push_back(pt_accessor.const_vector_attribute(vertices[i]));
        }
        if (wmtk::utils::wmtk_orient3d(pos[0], pos[1], pos[2], pos[3]) <= 0) {
            wmtk::logger().error("Flipped tet!");
        }
    }
    logger().info("Executing collapse ...");

    pre_stats = scheduler.run_operation_on_all(*collapse_then_round, visited_edge_flag.as<char>());
    logger().info(
        "Executed {}, {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, "
        "executing: {}",
        "preprocessing collapse",
        pre_stats.number_of_performed_operations(),
        pre_stats.number_of_successful_operations(),
        pre_stats.number_of_failed_operations(),
        pre_stats.collecting_time,
        pre_stats.sorting_time,
        pre_stats.executing_time);

    success = pre_stats.number_of_successful_operations();

    // compute max energy
    double max_energy = std::numeric_limits<double>::lowest();
    double min_energy = std::numeric_limits<double>::max();
    double avg_energy = 0;
    for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
        // double e = amips->get_value(simplex::Simplex(mesh->top_simplex_type(), t));
        double e = amips_accessor.scalar_attribute(t);
        max_energy = std::max(max_energy, e);
        min_energy = std::min(min_energy, e);
        avg_energy += e;
    }

    avg_energy = avg_energy / mesh->get_all(mesh->top_simplex_type()).size();

    logger().info(
        "Max AMIPS Energy: {}, Min AMIPS Energy: {}, Avg AMIPS Energy: {}",
        max_energy,
        min_energy,
        avg_energy);

    // std::ofstream file0("quality_plot_pre.csv");
    // file0 << "tid, quality" << std::endl;
    // int64_t t_cnt = 0;
    // for (const auto& t : mesh->get_all(PrimitiveType::Tetrahedron)) {
    //     t_cnt++;
    //     file0 << t_cnt << ", " << amips_accessor.scalar_attribute(t) << std::endl;
    // }


    double old_max_energy = max_energy;
    double old_avg_energy = avg_energy;
    int iii = 0;
    bool is_double = false;
    for (int64_t i = 0; i < options.passes; ++i) {
        logger().info("--------------------------- Pass {} ---------------------------", i);

        SchedulerStats pass_stats;
        int jj = 0;
        for (auto& op : ops) {
            for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
                const auto vertices = mesh->orient_vertices(t);
                std::vector<Vector3d> pos;
                for (int i = 0; i < vertices.size(); ++i) {
                    pos.push_back(pt_accessor.const_vector_attribute(vertices[i]));
                }
                if (wmtk::utils::wmtk_orient3d(pos[0], pos[1], pos[2], pos[3]) <= 0) {
                    wmtk::logger().error("Flipped tet!");
                }
            }
            logger().info("Executing {} ...", ops_name[jj]);
            SchedulerStats stats;
            if (op->primitive_type() == PrimitiveType::Edge) {
                stats = scheduler.run_operation_on_all(*op, visited_edge_flag.as<char>());
                // } else if (ops_name[jj] == "SMOOTHING") {
                //     // stats = scheduler.run_operation_on_all(*op);
                //     stats =
                //         scheduler.run_operation_on_all_coloring(*op,
                //         coloring_attribute.as<int64_t>());
            } else {
                stats = scheduler.run_operation_on_all(*op);
            }
            pass_stats += stats;
            logger().info(
                "Executed {}, {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, "
                "executing: {}",
                ops_name[jj],
                stats.number_of_performed_operations(),
                stats.number_of_successful_operations(),
                stats.number_of_failed_operations(),
                stats.collecting_time,
                stats.sorting_time,
                stats.executing_time);

            success = stats.number_of_successful_operations();

            avg_energy = 0;

            // compute max energy
            max_energy = std::numeric_limits<double>::lowest();
            min_energy = std::numeric_limits<double>::max();
            for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
                // double e = amips->get_value(simplex::Simplex(mesh->top_simplex_type(), t));
                double e = amips_accessor.scalar_attribute(t);
                max_energy = std::max(max_energy, e);
                min_energy = std::min(min_energy, e);
                avg_energy += e;
            }

            avg_energy = avg_energy / mesh->get_all(mesh->top_simplex_type()).size();

            logger().info(
                "Max AMIPS Energy: {}, Min AMIPS Energy: {}, Avg AMIPS Energy: {}",
                max_energy,
                min_energy,
                avg_energy);


            ++jj;
        }

        logger().info(
            "Executed {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
            pass_stats.number_of_performed_operations(),
            pass_stats.number_of_successful_operations(),
            pass_stats.number_of_failed_operations(),
            pass_stats.collecting_time,
            pass_stats.sorting_time,
            pass_stats.executing_time);

        multimesh::consolidate(*mesh);

        write(
            mesh,
            paths.output_dir,
            options.output,
            options.attributes.position,
            i + 1,
            options.intermediate_output);

        write(
            surface_mesh,
            paths.output_dir,
            options.output + "_intermediate_surface",
            options.attributes.position,
            i + 1,
            options.intermediate_output);

        assert(mesh->is_connectivity_valid());

        // compute max energy
        max_energy = std::numeric_limits<double>::lowest();
        min_energy = std::numeric_limits<double>::max();
        avg_energy = 0;
        for (const auto& t : mesh->get_all(mesh->top_simplex_type())) {
            // double e = amips->get_value(simplex::Simplex(mesh->top_simplex_type(), t));
            double e = amips_accessor.scalar_attribute(t);
            max_energy = std::max(max_energy, e);
            min_energy = std::min(min_energy, e);
            avg_energy += e;
        }

        avg_energy = avg_energy / mesh->get_all(mesh->top_simplex_type()).size();

        is_double = true;

        logger().info(
            "Max AMIPS Energy: {}, Min AMIPS Energy: {}, Avg AMIPS Energy: {}",
            max_energy,
            min_energy,
            avg_energy);


        // adjust sizing field
        if (i > 0 && old_max_energy - max_energy < 5e-1 &&
            (old_avg_energy - avg_energy) / avg_energy < 0.1) {
            wmtk::logger().info("adjusting sizing field ...");

            adjust_sizing_field(
                *mesh,
                pt_attribute.as<double>(),
                edge_length_attribute.as<double>(),
                sizing_field_scalar_attribute.as<double>(),
                amips_attribute.as<double>(),
                target_edge_length_attribute.as<double>(),
                visited_vertex_flag.as<char>(),
                target_max_amips,
                max_energy,
                target_edge_length,
                min_edge_length);

            wmtk::logger().info("adjusting sizing field finished");

            // wmtk::logger().info("setting energy filter ...");
            // set_operation_energy_filter_after_sizing_field(
            //     *mesh,
            //     pt_attribute.as<double>(),
            //     amips_attribute.as<double>(),
            //     energy_filter_attribute.as<char>(),
            //     visited_vertex_flag.as<char>(),
            //     target_max_amips,
            //     max_energy,
            //     target_edge_length);
            // wmtk::logger().info("setting energy filter finished");

            // int64_t e_cnt = 0;
            // for (const auto& e : mesh->get_all(PrimitiveType::Edge)) {
            //     if (energy_filter_accessor.scalar_attribute(e) == char(1) ||
            //         energy_filter_accessor.scalar_attribute(
            //             mesh->switch_tuple(e, PrimitiveType::Vertex)) == char(1)) {
            //         e_cnt++;
            //     }
            // }
            // wmtk::logger().info(
            //     "{} edges are going to be executed out of {}",
            //     e_cnt,
            //     mesh->get_all(PrimitiveType::Edge).size());

            for (const auto& v : mesh->get_all(PrimitiveType::Vertex)) {
                energy_filter_accessor.scalar_attribute(v) = char(1);
            }
            wmtk::logger().info("reset energy filter");
        } else {
            wmtk::logger().info("setting energy filter ...");
            set_operation_energy_filter(
                *mesh,
                pt_attribute.as<double>(),
                amips_attribute.as<double>(),
                energy_filter_attribute.as<char>(),
                visited_vertex_flag.as<char>(),
                target_max_amips,
                max_energy,
                target_edge_length);
            wmtk::logger().info("setting energy filter finished");

            int64_t e_cnt = 0;
            for (const auto& e : mesh->get_all(PrimitiveType::Edge)) {
                if (energy_filter_accessor.scalar_attribute(e) == char(1) ||
                    energy_filter_accessor.scalar_attribute(
                        mesh->switch_tuple(e, PrimitiveType::Vertex)) == char(1)) {
                    e_cnt++;
                }
            }
            wmtk::logger().info(
                "{} edges are going to be executed out of {}",
                e_cnt,
                mesh->get_all(PrimitiveType::Edge).size());
        }

        old_max_energy = max_energy;
        old_avg_energy = avg_energy;

        // stop at good quality
        if (max_energy <= target_max_amips && is_double) break;
    }

    // output
    cache.write_mesh(*mesh, options.output);
}
} // namespace wmtk::components