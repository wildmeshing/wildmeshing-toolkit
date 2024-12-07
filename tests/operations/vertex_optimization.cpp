#include <catch2/catch_test_macros.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/function/LocalNeighborsSumFunction.hpp>
#include <wmtk/function/PerSimplexAutodiffFunction.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/invariants/SimplexInversionInvariant.hpp>
#include <wmtk/operations/OptimizationSmoothing.hpp>
#include <wmtk/utils/Logger.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TetMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"
using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::simplex;
using namespace wmtk::operations;
using namespace wmtk::tests_3d;

// this is a test energy class for mapping one triangle to another triangle
namespace wmtk::function {
class SquareDistance : public PerSimplexAutodiffFunction
{
public:
    SquareDistance(
        const TriMesh& mesh,
        const attribute::TypedAttributeHandle<double>& attribute_handle,
        const attribute::TypedAttributeHandle<double>& target_attribute_handle)
        : PerSimplexAutodiffFunction(
              mesh,
              PrimitiveType::Vertex,
              attribute::MeshAttributeHandle(const_cast<TriMesh&>(mesh), attribute_handle))
        , m_target_attribute_accessor(mesh.create_const_accessor<double>(target_attribute_handle))
    {}
    ~SquareDistance() override = default;
    using DScalar = PerSimplexAutodiffFunction::DScalar;
    using DSVec = Eigen::VectorX<DScalar>;

protected:
    DScalar eval(const Simplex& domain_simplex, const std::vector<DSVec>& coordinates)
        const override
    {
        auto target_coordinates = get_coordinates(m_target_attribute_accessor, domain_simplex);
        DScalar r;
        for (size_t j = 0; j < 3; ++j) {
            auto c = coordinates[j];
            auto t = target_coordinates[j];
            r += (c - t).squaredNorm();
        }
        return r;
    }

    const attribute::Accessor<double> m_target_attribute_accessor;
};
} // namespace wmtk::function
TEST_CASE("vertex_optimization_Newton_Method")
{
    opt_logger().set_level(spdlog::level::off);

    DEBUG_TriMesh mesh = single_2d_nonequilateral_triangle_with_positions();
    auto handler = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    function::AMIPS per_tri_amips(mesh, handler);
    auto energy =
        std::make_shared<function::LocalNeighborsSumFunction>(mesh, handler, per_tri_amips);

    OptimizationSmoothing op(energy);
    op.add_invariant(
        std::make_shared<SimplexInversionInvariant<double>>(mesh, handler.as<double>()));
    Scheduler scheduler;

    // iterate all the vertices and find max gradnorm
    auto get_min_grad_norm = [&mesh, &energy]() -> double {
        std::vector<Tuple> tuples = mesh.get_all(PrimitiveType::Vertex);
        double min_grad_norm = std::numeric_limits<double>::max();
        for (const Tuple& tuple : tuples) {
            double grad_norm =
                energy->get_gradient(Simplex(mesh, PrimitiveType::Vertex, tuple)).norm();
            if (grad_norm < min_grad_norm) {
                min_grad_norm = grad_norm;
            }
        }
        return min_grad_norm;
    };

    while (get_min_grad_norm() > 1e-10) {
        auto stats = scheduler.run_operation_on_all(op);
        REQUIRE(stats.number_of_successful_operations() > 0);
    }
    const attribute::Accessor<double> pos = mesh.create_const_accessor<double, 2>(handler);
    Tuple tuple = mesh.tuple_from_face_id(0);
    auto uv0 = pos.const_vector_attribute(tuple);
    auto uv1 = pos.const_vector_attribute(mesh.switch_vertex(tuple));
    auto uv2 = pos.const_vector_attribute(mesh.switch_vertex(mesh.switch_edge(tuple)));

    CHECK((uv0 - uv1).norm() - (uv1 - uv2).norm() < 1e-6);
    CHECK((uv0 - uv1).norm() - (uv0 - uv2).norm() < 1e-6);
    CHECK((uv1 - uv2).norm() - (uv0 - uv2).norm() < 1e-6);
}

TEST_CASE("vertex_optimization_tet_amips")
{
    opt_logger().set_level(spdlog::level::off);

    TetMesh mesh = three_incident_tets_with_positions();
    auto handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    function::AMIPS amips(mesh, handle);
    auto energy = std::make_shared<function::LocalNeighborsSumFunction>(mesh, handle, amips);
    OptimizationSmoothing op(energy);

    Scheduler scheduler;

    // iterate all the vertices and find max gradnorm
    auto get_min_grad_norm = [&mesh, &energy]() -> double {
        std::vector<Tuple> tuples = mesh.get_all(PrimitiveType::Vertex);
        double min_grad_norm = std::numeric_limits<double>::max();
        for (const Tuple& tuple : tuples) {
            double grad_norm =
                energy->get_gradient(Simplex(mesh, PrimitiveType::Vertex, tuple)).norm();
            if (grad_norm < min_grad_norm) {
                min_grad_norm = grad_norm;
            }
        }
        return min_grad_norm;
    };

    while (get_min_grad_norm() > 1e-10) {
        auto stats = scheduler.run_operation_on_all(op);
        REQUIRE(stats.number_of_successful_operations() > 0);
    }
}

TEST_CASE("vertex_optimization_Gradient_Descent")
{
    opt_logger().set_level(spdlog::level::off);

    DEBUG_TriMesh mesh = single_2d_nonequilateral_triangle_with_positions();
    auto handle = mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    auto target_coordinate_handle =
        mesh.register_attribute<double>("target_coordinate", PrimitiveType::Vertex, 2);

    auto target_acc = mesh.create_accessor<double, 2>(target_coordinate_handle);

    target_acc.vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, 0)) << 0, 0;
    target_acc.vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, 1)) << 1, 0;
    target_acc.vector_attribute(mesh.tuple_from_id(PrimitiveType::Vertex, 2)) << 0, 1;

    function::SquareDistance squared_dist(
        mesh,
        handle.as<double>(),
        target_coordinate_handle.as<double>());
    auto energy = std::make_shared<function::LocalNeighborsSumFunction>(mesh, handle, squared_dist);
    OptimizationSmoothing op(energy);

    // iterate all the vertices and find max gradnorm
    auto get_min_grad_norm = [&mesh, &energy]() -> double {
        std::vector<Tuple> tuples = mesh.get_all(PrimitiveType::Vertex);
        double min_grad_norm = std::numeric_limits<double>::max();
        for (const Tuple& tuple : tuples) {
            Eigen::Vector2d grad =
                energy->get_gradient(Simplex(mesh, PrimitiveType::Vertex, tuple));
            double grad_norm = grad.norm();
            if (grad_norm < min_grad_norm) {
                min_grad_norm = grad_norm;
            }
        }
        return min_grad_norm;
    };

    SchedulerStats stats;
    Scheduler scheduler;

    do {
        stats = scheduler.run_operation_on_all(op);
    } while (get_min_grad_norm() > 1e-3 && stats.number_of_successful_operations() > 0);
    const attribute::Accessor<double> pos = mesh.create_const_accessor<double, 2>(handle);
    Tuple tuple = mesh.tuple_from_face_id(0);
    Eigen::Vector2d uv0 = pos.const_vector_attribute(tuple);
    Eigen::Vector2d uv1 = pos.const_vector_attribute(mesh.switch_vertex(tuple));
    Eigen::Vector2d uv2 = pos.const_vector_attribute(mesh.switch_vertex(mesh.switch_edge(tuple)));


    // CHECK((uv0 - uv1).norm() - (uv1 - uv2).norm() < 1e-6);
    // CHECK((uv0 - uv1).norm() - (uv0 - uv2).norm() < 1e-6);
    // CHECK((uv1 - uv2).norm() - (uv0 - uv2).norm() < 1e-6);
}

TEST_CASE("vertex_optimization_with_coloring", "[operations][parallel]")
{
    opt_logger().set_level(spdlog::level::off);
    logger().set_level(spdlog::level::off);

    auto run = [](DEBUG_TriMesh& m, bool parallel_execution) {
        auto color_handle =
            m.register_attribute<int64_t>("coloring", PrimitiveType::Vertex, 1, false, -1);

        Scheduler scheduler;
        const int64_t max_colors = scheduler.color_vertices(color_handle);

        auto acc = m.create_const_accessor<int64_t>(color_handle);

        CHECK(acc.const_scalar_attribute(m.vertex_tuple_from_id(0)) == 0);
        CHECK(acc.const_scalar_attribute(m.vertex_tuple_from_id(1)) == 1);
        CHECK(acc.const_scalar_attribute(m.vertex_tuple_from_id(2)) == 2);

        auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

        function::AMIPS per_tri_amips(m, pos_handle);
        auto energy =
            std::make_shared<function::LocalNeighborsSumFunction>(m, pos_handle, per_tri_amips);

        OptimizationSmoothing op(energy);
        op.add_invariant(
            std::make_shared<SimplexInversionInvariant<double>>(m, pos_handle.as<double>()));

        auto stats = scheduler.run_operation_on_all_with_coloring(
            op,
            color_handle,
            max_colors,
            parallel_execution);
        REQUIRE(stats.number_of_successful_operations() > 0);
    };

    SECTION("single_triangle_coloring")
    {
        DEBUG_TriMesh mesh_1 = single_2d_nonequilateral_triangle_with_positions();
        DEBUG_TriMesh mesh_2 = single_2d_nonequilateral_triangle_with_positions();

        run(mesh_1, false); // sequential
        run(mesh_2, true); // parallel

        {
            auto color_handle =
                mesh_1.get_attribute_handle<int64_t>("coloring", PrimitiveType::Vertex);

            Scheduler scheduler;
            const int64_t max_colors = scheduler.color_vertices(color_handle);

            auto acc = mesh_1.create_const_accessor<int64_t>(color_handle);

            CHECK(acc.const_scalar_attribute(mesh_1.vertex_tuple_from_id(0)) == 0);
            CHECK(acc.const_scalar_attribute(mesh_1.vertex_tuple_from_id(1)) == 1);
            CHECK(acc.const_scalar_attribute(mesh_1.vertex_tuple_from_id(2)) == 2);
        }

        auto pos_handle_1 = mesh_1.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        auto pos_handle_2 = mesh_2.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        const auto pos_acc_1 = mesh_1.create_const_accessor<double>(pos_handle_1);
        const auto pos_acc_2 = mesh_2.create_const_accessor<double>(pos_handle_2);

        const auto vertices = mesh_1.get_all(PrimitiveType::Vertex);
        /*
         * This is a bit hacky because I am using the same tuples in two different meshes. But as
         * they are exactly the same, it is fine here.
         */
        for (const Tuple& t : vertices) {
            const auto p1 = pos_acc_1.const_vector_attribute(t);
            const auto p2 = pos_acc_2.const_vector_attribute(t);
            CHECK(p1 == p2);
        }
    }
    SECTION("edge_region_coloring")
    {
        DEBUG_TriMesh mesh = edge_region();

        auto color_handle =
            mesh.register_attribute<int64_t>("coloring", PrimitiveType::Vertex, 1, false, -1);

        Scheduler scheduler;
        scheduler.color_vertices(color_handle);

        auto acc = mesh.create_const_accessor<int64_t>(color_handle);

        CHECK(acc.const_scalar_attribute(mesh.vertex_tuple_from_id(0)) == 0);
        CHECK(acc.const_scalar_attribute(mesh.vertex_tuple_from_id(1)) == 1);
        CHECK(acc.const_scalar_attribute(mesh.vertex_tuple_from_id(2)) == 0);
        CHECK(acc.const_scalar_attribute(mesh.vertex_tuple_from_id(3)) == 1);
        CHECK(acc.const_scalar_attribute(mesh.vertex_tuple_from_id(4)) == 2);
        CHECK(acc.const_scalar_attribute(mesh.vertex_tuple_from_id(5)) == 3);
        CHECK(acc.const_scalar_attribute(mesh.vertex_tuple_from_id(6)) == 1);
        CHECK(acc.const_scalar_attribute(mesh.vertex_tuple_from_id(7)) == 0);
        CHECK(acc.const_scalar_attribute(mesh.vertex_tuple_from_id(8)) == 1);
        CHECK(acc.const_scalar_attribute(mesh.vertex_tuple_from_id(9)) == 0);
    }
}