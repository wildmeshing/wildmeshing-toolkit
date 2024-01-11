#include "ATOperations.hpp"
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleAnalyticalIntegral.hpp>
#include <wmtk/components/adaptive_tessellation/function/simplex/PerTriangleTextureIntegralAccuracyFunction.hpp>
#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/function/simplex/TriangleAMIPS.hpp>

#include <wmtk/invariants/FunctionInvariant.hpp>
#include <wmtk/invariants/InteriorEdgeInvariant.hpp>
#include <wmtk/invariants/InteriorVertexInvariant.hpp>
#include <wmtk/invariants/MultiMeshLinkConditionInvariant.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>

#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/EdgeSplit.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/operations/composite/TriEdgeSwap.hpp>

#include <wmtk/simplex/Simplex.hpp>

#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/Scheduler.hpp>

#include "ATOptions.hpp"

#include <wmtk/components/adaptive_tessellation/function/utils/ThreeChannelPositionMapEvaluator.hpp>
namespace wmtk::components::operations::internal {
using namespace wmtk::operations;
// using namespace operations::tri_mesh;
using namespace wmtk::operations::composite;
using namespace wmtk::function;
using namespace wmtk::invariants;

ATOperations::ATOperations(ATData& atdata, double target_edge_length)
    : m_atdata(atdata)
    , m_evaluator(m_atdata.funcs())
    , m_edge_length_accessor(
          m_atdata.uv_mesh().create_accessor(m_atdata.m_3d_edge_length_handle.as<double>()))

{
    m_ops.clear();
    auto compute_edge_length = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };
    m_edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            m_atdata.m_3d_edge_length_handle,
            m_atdata.m_position_handle,
            compute_edge_length);
    // Lambdas for priority
    m_long_edges_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({-m_edge_length_accessor.scalar_attribute(s.tuple())});
    };
    m_short_edges_first = [&](const Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({m_edge_length_accessor.scalar_attribute(s.tuple())});
    };
    const auto vertices = m_atdata.uv_mesh_ptr()->get_all(PrimitiveType::Vertex);
    auto pt_accessor = m_atdata.uv_mesh_ptr()->create_accessor(m_atdata.m_uv_handle.as<double>());
    auto vert_pos_accessor =
        m_atdata.uv_mesh_ptr()->create_accessor(m_atdata.m_position_handle.as<double>());

    auto compute_vertex_position = [&](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 1);
        assert(P.rows() == 2);
        Eigen::Vector2d uv = P.col(0);
        return m_evaluator.uv_to_position(uv);
    };
    m_3d_position_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            m_atdata.m_position_handle,
            m_atdata.m_uv_handle,
            compute_vertex_position);
    // initialize this position field
    for (const auto& v : vertices) {
        const auto p = pt_accessor.vector_attribute(v);
        vert_pos_accessor.vector_attribute(v) = compute_vertex_position(p);
    }

    //////////////////////////////////
    // computng bbox diagonal
    bool planar =
        false; // TODO this needs to be read from the options. For now we work only on uv mesh
    Eigen::VectorXd bmin(planar ? 2 : 3);
    bmin.setConstant(std::numeric_limits<double>::max());
    Eigen::VectorXd bmax(planar ? 2 : 3);
    bmax.setConstant(std::numeric_limits<double>::min());
    for (const auto& v : vertices) {
        const auto p = vert_pos_accessor.vector_attribute(v);
        for (int64_t d = 0; d < bmax.size(); ++d) {
            bmin[d] = std::min(bmin[d], p[d]);
            bmax[d] = std::max(bmax[d], p[d]);
        }
    }

    const double bbdiag = (bmax - bmin).norm();
    m_target_edge_length = target_edge_length * bbdiag;
}

void ATOperations::set_energies()
{
    m_accuracy_energy = std::make_shared<wmtk::function::PerTriangleAnalyticalIntegral>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle(),
        m_atdata.funcs());
    m_amips_energy = std::make_shared<wmtk::function::TriangleAMIPS>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.uv_handle());
    m_3d_amips_energy = std::make_shared<wmtk::function::PositionMapAMIPS>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.m_uv_handle,
        m_evaluator);

    m_sum_energy = std::make_shared<wmtk::function::SumEnergy>(
        *m_atdata.uv_mesh_ptr(),
        m_atdata.m_uv_handle,
        m_evaluator);
}

void ATOperations::AT_smooth_interior()
{
    auto& uv_mesh = m_atdata.uv_mesh();
    auto uv_handle = m_atdata.uv_handle();
    // Energy to optimize
    std::shared_ptr<wmtk::function::PerTriangleTextureIntegralAccuracyFunction> accuracy =
        std::make_shared<wmtk::function::PerTriangleTextureIntegralAccuracyFunction>(
            uv_mesh,
            uv_handle,
            m_atdata.images());

    // std::shared_ptr<wmtk::function::TriangleAMIPS> amips =
    //     std::make_shared<wmtk::function::TriangleAMIPS>(m_atdata.uv_mesh(),
    //     m_atdata.uv_handle());
    std::shared_ptr<wmtk::function::AMIPS> amips =
        std::make_shared<wmtk::function::AMIPS>(m_atdata.uv_mesh(), m_atdata.uv_handle());
    amips->attribute_handle();
    // for (auto& f : uv_mesh.get_all(PrimitiveType::Face)) {
    //     auto val = accuracy->get_value(Simplex::face(f));
    //     std::cout << " has value " << val << std::endl;
    // }

    // MeshAttributeHandle handle = amips->attribute_handle();
    // assert(handle.is_valid());
    std::shared_ptr<wmtk::function::LocalNeighborsSumFunction> energy =
        std::make_shared<wmtk::function::LocalNeighborsSumFunction>(
            m_atdata.uv_mesh(),
            m_atdata.uv_handle(),
            *amips);
    for (auto& v : uv_mesh.get_all(PrimitiveType::Vertex)) {
        energy->get_value(Simplex::vertex(v));
        break;
    }
    m_ops.emplace_back(std::make_shared<wmtk::operations::OptimizationSmoothing>(energy));
    m_ops.back()->add_invariant(
        std::make_shared<SimplexInversionInvariant>(uv_mesh, uv_handle.as<double>()));
    m_ops.back()->add_invariant(std::make_shared<InteriorVertexInvariant>(uv_mesh));
    m_ops.back()->add_transfer_strategy(m_edge_length_update);
    m_ops.back()->use_random_priority() = true;
}

void ATOperations::AT_smooth_interior(
    std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr)
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();
    wmtk::attribute::MeshAttributeHandle uv_handle = m_atdata.uv_handle();

    std::shared_ptr<wmtk::function::LocalNeighborsSumFunction> energy =
        std::make_shared<wmtk::function::LocalNeighborsSumFunction>(
            *uv_mesh_ptr,
            uv_handle,
            *function_ptr);

    m_ops.emplace_back(std::make_shared<wmtk::operations::OptimizationSmoothing>(energy));
    m_ops.back()->add_invariant(
        std::make_shared<SimplexInversionInvariant>(*uv_mesh_ptr, uv_handle.as<double>()));
    m_ops.back()->add_invariant(std::make_shared<InteriorVertexInvariant>(*uv_mesh_ptr));
    m_ops.back()->add_transfer_strategy(m_edge_length_update);
    m_ops.back()->add_transfer_strategy(m_3d_position_update);
    m_ops.back()->use_random_priority() = true;
}


void ATOperations::AT_split_interior()
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();
    wmtk::attribute::MeshAttributeHandle uv_handle = m_atdata.uv_handle();

    // 1) EdgeSplit
    auto split = std::make_shared<wmtk::operations::EdgeSplit>(*uv_mesh_ptr);
    split->add_invariant(std::make_shared<TodoLargerInvariant>(
        *uv_mesh_ptr,
        m_atdata.m_3d_edge_length_handle.as<double>(),
        4.0 / 3.0 * m_target_edge_length));
    // split->add_invariant(std::make_shared<InteriorEdgeInvariant>(uv_mesh));
    split->set_priority(m_long_edges_first);

    split->set_new_attribute_strategy(m_atdata.m_uv_handle);
    split->set_new_attribute_strategy(m_atdata.m_position_handle);
    split->set_new_attribute_strategy(m_atdata.m_3d_edge_length_handle);

    split->add_transfer_strategy(m_3d_position_update);
    split->add_transfer_strategy(m_edge_length_update);

    m_ops.emplace_back(split);
}

void ATOperations::AT_split_single_edge_mesh(Mesh* edge_meshi_ptr)
{
    auto m_t_handle = edge_meshi_ptr->get_attribute_handle<double>("t", PrimitiveType::Vertex);
    auto split = std::make_shared<wmtk::operations::EdgeSplit>(*edge_meshi_ptr);
    split->add_invariant(std::make_shared<TodoLargerInvariant>(
        m_atdata.uv_mesh(),
        m_atdata.m_3d_edge_length_handle.as<double>(),
        4.0 / 3.0 * m_target_edge_length));
    split->set_priority(m_long_edges_first);

    split->set_new_attribute_strategy(m_atdata.m_3d_edge_length_handle);
    split->set_new_attribute_strategy(m_atdata.m_uv_handle);
    split->set_new_attribute_strategy(m_t_handle);

    split->add_transfer_strategy(m_edge_length_update);
    m_ops.emplace_back(split);
}

void ATOperations::AT_split_boundary()
{
    auto& uv_mesh = m_atdata.uv_mesh();
    auto uv_handle = m_atdata.uv_handle();
    int64_t num_edge_meshes = m_atdata.num_edge_meshes();

    // 1) EdgeSplit on boundary
    for (int64_t i = 0; i < num_edge_meshes; ++i) {
        std::shared_ptr<Mesh> edge_meshi_ptr = m_atdata.edge_mesh_i_ptr(i);
        Mesh* sibling_mesh_ptr = m_atdata.sibling_edge_mesh_ptr(edge_meshi_ptr.get());
        AT_split_single_edge_mesh(edge_meshi_ptr.get());
        if (sibling_mesh_ptr == nullptr) {
            continue;
        }
        AT_split_single_edge_mesh(sibling_mesh_ptr);
    }
}

void ATOperations::AT_collapse_interior(
    std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr)
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();
    auto collapse = std::make_shared<wmtk::operations::EdgeCollapse>(*uv_mesh_ptr);
    collapse->add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(*uv_mesh_ptr));
    collapse->add_invariant(std::make_shared<InteriorEdgeInvariant>(*uv_mesh_ptr));
    collapse->add_invariant(std::make_shared<SimplexInversionInvariant>(
        *uv_mesh_ptr,
        m_atdata.uv_handle().as<double>()));
    collapse->add_invariant(
        std::make_shared<FunctionInvariant>(uv_mesh_ptr->top_simplex_type(), function_ptr));
    collapse->add_invariant(std::make_shared<TodoSmallerInvariant>(
        *uv_mesh_ptr,
        m_atdata.m_3d_edge_length_handle.as<double>(),
        4.0 / 5.0 * m_target_edge_length));
    collapse->set_priority(m_short_edges_first);

    auto clps_strat = std::make_shared<wmtk::operations::CollapseNewAttributeStrategy<double>>(
        m_atdata.uv_handle());
    clps_strat->set_simplex_predicate(wmtk::operations::BasicSimplexPredicate::IsInterior);
    clps_strat->set_strategy(wmtk::operations::CollapseBasicStrategy::Default);

    collapse->set_new_attribute_strategy(m_atdata.uv_handle(), clps_strat);

    auto clps_strat2 = std::make_shared<wmtk::operations::CollapseNewAttributeStrategy<double>>(
        m_atdata.m_position_handle);
    clps_strat2->set_simplex_predicate(wmtk::operations::BasicSimplexPredicate::IsInterior);
    clps_strat2->set_strategy(wmtk::operations::CollapseBasicStrategy::Default);
    collapse->set_new_attribute_strategy(m_atdata.m_position_handle, clps_strat2);

    collapse->set_new_attribute_strategy(m_atdata.m_3d_edge_length_handle);
    // collapse->set_new_attribute_strategy(face_error_attribute);

    collapse->add_transfer_strategy(m_3d_position_update);
    collapse->add_transfer_strategy(m_edge_length_update);
    // collapse->add_transfer_strategy(face_error_update);
    m_ops.emplace_back(collapse);
}

void ATOperations::AT_swap_interior(
    std::shared_ptr<wmtk::function::PerSimplexFunction> function_ptr)
{
    std::shared_ptr<Mesh> uv_mesh_ptr = m_atdata.uv_mesh_ptr();
    auto swap = std::make_shared<TriEdgeSwap>(*uv_mesh_ptr);
    swap->collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(*uv_mesh_ptr));
    swap->add_invariant(std::make_shared<InteriorEdgeInvariant>(*uv_mesh_ptr));
    swap->add_invariant(std::make_shared<SimplexInversionInvariant>(
        *uv_mesh_ptr,
        m_atdata.uv_handle().as<double>()));
    swap->add_invariant(
        std::make_shared<FunctionInvariant>(uv_mesh_ptr->top_simplex_type(), function_ptr));
    swap->set_priority(m_long_edges_first);

    swap->split().set_new_attribute_strategy(m_atdata.uv_handle());
    swap->collapse().set_new_attribute_strategy(
        m_atdata.uv_handle(),
        wmtk::operations::CollapseBasicStrategy::CopyOther);

    swap->split().set_new_attribute_strategy(m_atdata.m_position_handle);
    swap->collapse().set_new_attribute_strategy(
        m_atdata.m_position_handle,
        wmtk::operations::CollapseBasicStrategy::CopyOther);

    swap->split().set_new_attribute_strategy(m_atdata.m_3d_edge_length_handle);
    swap->collapse().set_new_attribute_strategy(
        m_atdata.m_3d_edge_length_handle,
        wmtk::operations::CollapseBasicStrategy::CopyOther);

    swap->add_transfer_strategy(m_3d_position_update);
    swap->add_transfer_strategy(m_edge_length_update);

    m_ops.push_back(swap);
}

void ATOperations::at_operation(const nlohmann::json& j)
{
    //////////////////////////////////
    // Load mesh from settings
    ATOptions options = j.get<ATOptions>();
    const std::filesystem::path& file = options.input;
    std::shared_ptr<Mesh> mesh = m_atdata.uv_mesh_ptr();

    //////////////////////////////////
    // Storing edge lengths
    auto edge_length_attribute = m_atdata.edge_len_handle();
    auto edge_length_accessor = mesh->create_accessor(edge_length_attribute.as<double>());

    auto vert_pos_attribute =
        mesh->register_attribute<double>("vert_pos", PrimitiveType::Vertex, 3);
    auto vert_pos_accessor = mesh->create_accessor(vert_pos_attribute.as<double>());

    //////////////////////////////////
    // Retriving vertices
    auto pt_attribute = mesh->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pt_accessor = mesh->create_accessor(pt_attribute.as<double>());

    // Edge length update
    auto compute_edge_length = [](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 2);
        assert(P.rows() == 2 || P.rows() == 3);
        return Eigen::VectorXd::Constant(1, (P.col(0) - P.col(1)).norm());
    };
    auto edge_length_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            edge_length_attribute,
            vert_pos_attribute,
            compute_edge_length);

    //////////////////////////////////
    // computing edge lengths
    const auto edges = mesh->get_all(PrimitiveType::Edge);
    for (const auto& e : edges) {
        const auto p0 = vert_pos_accessor.vector_attribute(e);
        const auto p1 = vert_pos_accessor.vector_attribute(mesh->switch_vertex(e));

        edge_length_accessor.scalar_attribute(e) = (p0 - p1).norm();
    }

    //////////////////////////////////
    // computng bbox diagonal
    Eigen::VectorXd bmin(options.planar ? 2 : 3);
    bmin.setConstant(std::numeric_limits<double>::max());
    Eigen::VectorXd bmax(options.planar ? 2 : 3);
    bmax.setConstant(std::numeric_limits<double>::min());

    const auto vertices = mesh->get_all(PrimitiveType::Vertex);
    for (const auto& v : vertices) {
        const auto p = pt_accessor.vector_attribute(v);
        for (int64_t d = 0; d < bmax.size(); ++d) {
            bmin[d] = std::min(bmin[d], p[d]);
            bmax[d] = std::max(bmax[d], p[d]);
        }
    }

    const double bbdiag = (bmax - bmin).norm();
    const double target_edge_length = options.target_edge_length * bbdiag;

    //////////////////////////////////
    // Lambdas for priority
    auto long_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({-edge_length_accessor.scalar_attribute(s.tuple())});
    };
    auto short_edges_first = [&](const simplex::Simplex& s) {
        assert(s.primitive_type() == PrimitiveType::Edge);
        return std::vector<double>({edge_length_accessor.scalar_attribute(s.tuple())});
    };

    //////////////////////////////////
    // // Energy to optimize
    // std::shared_ptr<wmtk::function::PerSimplexFunction> amips =
    //     std::make_shared<wmtk::function::AMIPS>(*mesh, pt_attribute);


    std::shared_ptr<wmtk::function::TriangleAMIPS> amips =
        std::make_shared<wmtk::function::TriangleAMIPS>(*mesh, pt_attribute);

    std::array<std::shared_ptr<image::Image>, 3> images = {
        {std::make_shared<image::Image>(500, 500),
         std::make_shared<image::Image>(500, 500),
         std::make_shared<image::Image>(500, 500)}};
    auto u = [](const double& u, [[maybe_unused]] const double& v) -> double { return u; };
    auto v = []([[maybe_unused]] const double& u, const double& v) -> double { return v; };
    std::function<double(double, double)> height_function =
        [](const double& u, [[maybe_unused]] const double& v) -> double {
        // return u * u + v * v;
        return sin(2 * M_PI * u) * cos(2 * M_PI * v);
    };
    images[0]->set(u);
    images[1]->set(v);
    images[2]->set(height_function);
    std::shared_ptr<wmtk::function::PerTriangleTextureIntegralAccuracyFunction> texture =
        std::make_shared<wmtk::function::PerTriangleTextureIntegralAccuracyFunction>(
            *mesh,
            pt_attribute,
            images);

    std::array<std::shared_ptr<image::SamplingAnalyticFunction>, 3> funcs = {
        {std::make_shared<image::SamplingAnalyticFunction>(
             image::SamplingAnalyticFunction_FunctionType::Linear,
             1,
             0,
             0.),
         std::make_shared<image::SamplingAnalyticFunction>(
             image::SamplingAnalyticFunction_FunctionType::Linear,
             0,
             1,
             0.),
         std::make_shared<image::SamplingAnalyticFunction>(
             image::SamplingAnalyticFunction_FunctionType::Periodic,
             2,
             2,
             1.)}
        //  std::make_shared<image::SamplingAnalyticFunction>(
        //      image::SamplingAnalyticFunction_FunctionType::Linear,
        //      0,
        //      0,
        //      0.)}

    };
    std::shared_ptr<wmtk::function::PerTriangleAnalyticalIntegral> accuracy =
        std::make_shared<wmtk::function::PerTriangleAnalyticalIntegral>(*mesh, pt_attribute, funcs);

    // vertex position update

    function::utils::ThreeChannelPositionMapEvaluator evaluator(funcs);
    auto compute_vertex_position = [&evaluator](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
        assert(P.cols() == 1);
        assert(P.rows() == 2);
        Eigen::Vector2d uv = P.col(0);
        return evaluator.uv_to_position(uv);
    };
    auto vert_position_update =
        std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
            vert_pos_attribute,
            pt_attribute,
            compute_vertex_position);


    /*{ // face error update
        auto face_error_attribute =
            mesh->register_attribute<double>("face_error", PrimitiveType::Face, 1);
        auto face_error_accessor = mesh->create_accessor(face_error_attribute.as<double>());

        auto compute_face_error = [&evaluator](const Eigen::MatrixXd& P) -> Eigen::VectorXd {
            assert(P.cols() == 3);
            assert(P.rows() == 2);
            AT::function::utils::AnalyticalFunctionTriangleQuadrature analytical_quadrature(
                evaluator);
            Eigen::Vector2<double> uv0 = P.col(0);
            Eigen::Vector2<double> uv1 = P.col(1);
            Eigen::Vector2<double> uv2 = P.col(2);
            Eigen::VectorXd error(1);
            error(0) = analytical_quadrature.get_error_one_triangle_exact(uv0, uv1, uv2);
            return error;
        };
        auto face_error_update =
            std::make_shared<wmtk::operations::SingleAttributeTransferStrategy<double, double>>(
                face_error_attribute,
                pt_attribute,
                compute_face_error);
        for (auto& f : mesh->get_all(PrimitiveType::Face)) {
            if (!mesh->is_ccw(f)) {
                f = mesh->switch_vertex(f);
            }
            const Eigen::Vector2d v0 = pt_accessor.vector_attribute(f);
            const Eigen::Vector2d v1 = pt_accessor.vector_attribute(mesh->switch_vertex(f));
            const Eigen::Vector2d v2 =
                pt_accessor.vector_attribute(mesh->switch_vertex(mesh->switch_edge(f)));
            AT::function::utils::AnalyticalFunctionTriangleQuadrature analytical_quadrature(
                evaluator);

            auto res = analytical_quadrature.get_error_one_triangle_exact(v0, v1, v2);
            face_error_accessor.scalar_attribute(f) = res;
        }
    }*/

    // initialize this two fields
    for (const auto& v : vertices) {
        const auto p = pt_accessor.vector_attribute(v);
        vert_pos_accessor.vector_attribute(v) = compute_vertex_position(p);
    }

    opt_logger().set_level(spdlog::level::level_enum::critical);


    //////////////////////////////////
    // Creation of the 4 ops
    std::vector<std::shared_ptr<wmtk::operations::Operation>> ops;


    // 1) EdgeSplit
    auto split = std::make_shared<wmtk::operations::EdgeSplit>(*mesh);
    split->add_invariant(std::make_shared<TodoLargerInvariant>(
        *mesh,
        edge_length_attribute.as<double>(),
        4.0 / 3.0 * target_edge_length));
    split->set_priority(long_edges_first);

    split->set_new_attribute_strategy(edge_length_attribute);
    split->set_new_attribute_strategy(pt_attribute);
    split->set_new_attribute_strategy(vert_pos_attribute);
    // split->set_new_attribute_strategy(face_error_attribute);

    split->add_transfer_strategy(vert_position_update);
    // split->add_transfer_strategy(face_error_update);
    split->add_transfer_strategy(m_edge_length_update);
    ops.emplace_back(split);


    // 2) EdgeCollapse
    auto collapse = std::make_shared<wmtk::operations::EdgeCollapse>(*mesh);
    // invar_col -> add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(*mesh));


    // collapse-> add_invariant(invar_col);
    collapse->add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(*mesh));
    collapse->add_invariant(std::make_shared<InteriorEdgeInvariant>(*mesh));
    collapse->add_invariant(
        std::make_shared<SimplexInversionInvariant>(*mesh, pt_attribute.as<double>()));
    collapse->add_invariant(
        std::make_shared<FunctionInvariant>(mesh->top_simplex_type(), accuracy));
    collapse->add_invariant(std::make_shared<TodoSmallerInvariant>(
        *mesh,
        edge_length_attribute.as<double>(),
        4.0 / 5.0 * target_edge_length));
    collapse->set_priority(short_edges_first);

    auto clps_strat = std::make_shared<CollapseNewAttributeStrategy<double>>(pt_attribute);
    clps_strat->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
    clps_strat->set_strategy(CollapseBasicStrategy::Default);

    collapse->set_new_attribute_strategy(pt_attribute, clps_strat);
    collapse->set_new_attribute_strategy(edge_length_attribute);

    collapse->add_transfer_strategy(m_edge_length_update);

    auto clps_strat2 = std::make_shared<CollapseNewAttributeStrategy<double>>(vert_pos_attribute);
    clps_strat2->set_simplex_predicate(BasicSimplexPredicate::IsInterior);
    clps_strat2->set_strategy(CollapseBasicStrategy::Default);

    collapse->set_new_attribute_strategy(vert_pos_attribute, clps_strat2);
    // collapse->set_new_attribute_strategy(face_error_attribute);

    collapse->add_transfer_strategy(vert_position_update);
    // collapse->add_transfer_strategy(face_error_update);
    ops.emplace_back(collapse);


    // 3) TriEdgeSwap
    if (mesh->top_simplex_type() == PrimitiveType::Face) {
        auto swap = std::make_shared<TriEdgeSwap>(*mesh);
        swap->collapse().add_invariant(std::make_shared<MultiMeshLinkConditionInvariant>(*mesh));
        swap->add_invariant(std::make_shared<InteriorEdgeInvariant>(*mesh));
        swap->add_invariant(
            std::make_shared<SimplexInversionInvariant>(*mesh, pt_attribute.as<double>()));
        swap->add_invariant(
            std::make_shared<FunctionInvariant>(mesh->top_simplex_type(), accuracy));
        swap->set_priority(long_edges_first);

        swap->collapse().set_new_attribute_strategy(vert_pos_attribute);
        swap->split().set_new_attribute_strategy(vert_pos_attribute);
        swap->collapse().set_new_attribute_strategy(edge_length_attribute);
        swap->split().set_new_attribute_strategy(edge_length_attribute);

        swap->split().set_new_attribute_strategy(pt_attribute);
        swap->collapse().set_new_attribute_strategy(pt_attribute, CollapseBasicStrategy::CopyOther);
        swap->split().set_new_attribute_strategy(vert_pos_attribute);
        swap->collapse().set_new_attribute_strategy(
            vert_pos_attribute,
            CollapseBasicStrategy::CopyOther);

        swap->add_transfer_strategy(edge_length_update);

        ops.push_back(swap);
    } else // if (mesh->top_simplex_type() == PrimitiveType::Face) {
    {
        throw std::runtime_error("unsupported");
    }

    // 4) Smoothing
    auto energy =
        std::make_shared<wmtk::function::LocalNeighborsSumFunction>(*mesh, pt_attribute, *accuracy);
    ops.emplace_back(std::make_shared<OptimizationSmoothing>(energy));
    ops.back()->add_invariant(
        std::make_shared<SimplexInversionInvariant>(*mesh, pt_attribute.as<double>()));
    ops.back()->add_invariant(std::make_shared<InteriorVertexInvariant>(*mesh));
    ops.back()->add_transfer_strategy(edge_length_update);
    ops.back()->use_random_priority() = true;


    //////////////////////////////////
    // Running all ops in order n times
    Scheduler scheduler;
    for (int64_t i = 0; i < 20; ++i) {
        logger().info("Pass {}", i);
        SchedulerStats pass_stats;
        for (auto& op : ops) pass_stats += scheduler.run_operation_on_all(*op);

        logger().info(
            "Executed {} ops (S/F) {}/{}. Time: collecting: {}, sorting: {}, executing: {}",
            pass_stats.number_of_performed_operations(),
            pass_stats.number_of_successful_operations(),
            pass_stats.number_of_failed_operations(),
            pass_stats.collecting_time,
            pass_stats.sorting_time,
            pass_stats.executing_time);
        // write(mesh, options.filename, i + 1, options.intermediate_output);
    }
    // write(mesh, "no_operation", 0, options.intermediate_output);
    // const std::filesystem::path data_dir = "";
    // wmtk::io::ParaviewWriter
    //     writer(data_dir / ("output_pos"), "vert_pos", *mesh, true, true, true, false);
    // mesh->serialize(writer);
}

} // namespace wmtk::components::operations::internal