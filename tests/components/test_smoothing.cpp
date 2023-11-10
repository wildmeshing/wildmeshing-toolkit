#include <catch2/catch_test_macros.hpp>
#include <wmtk/Primitive.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/Simplex.hpp>
#include <wmtk/function/AMIPS.hpp>
#include <wmtk/function/LocalDifferentiableFunction.hpp>
#include <wmtk/operations/tri_mesh/VertexSmoothUsingDifferentiableEnergy.hpp>
#include <wmtk/utils/Logger.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"
using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::operations;

TEST_CASE("smoothing_Newton_Method")
{
    DEBUG_TriMesh mesh = single_2d_triangle_with_random_positions();
    OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy> op_settings;
    op_settings.coordinate_handle =
        mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    op_settings.smooth_boundary = true;
    op_settings.second_order = true;
    op_settings.line_search = false;
    op_settings.step_size = 0.1;
    std::shared_ptr<function::AMIPS> per_tri_amips = std::make_shared<function::AMIPS>(
        mesh,
        mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex));
    op_settings.energy = std::make_unique<function::LocalDifferentiableFunction>(per_tri_amips);
    op_settings.initialize_invariants(mesh);

    Scheduler scheduler(mesh);
    const auto& factory =
        scheduler.add_operation_type<operations::tri_mesh::VertexSmoothUsingDifferentiableEnergy>(
            "optimize_vertices",
            std::move(op_settings));
    // iterate all the vertices and find max gradnorm
    auto get_min_grad_norm = [&mesh, &factory]() -> double {
        std::vector<Tuple> tuples = mesh.get_all(PrimitiveType::Vertex);
        double min_grad_norm = std::numeric_limits<double>::max();
        for (const Tuple& tuple : tuples) {
            double grad_norm = factory.settings()
                                   .energy->get_gradient(Simplex(PrimitiveType::Vertex, tuple))
                                   .norm();
            if (grad_norm < min_grad_norm) {
                min_grad_norm = grad_norm;
            }
        }
        return min_grad_norm;
    };

    while (get_min_grad_norm() > 1e-10) {
        scheduler.run_operation_on_all(PrimitiveType::Vertex, "optimize_vertices");
        REQUIRE(scheduler.number_of_successful_operations() > 0);
    }
    ConstAccessor<double> pos = mesh.create_const_accessor(op_settings.coordinate_handle);
    Tuple tuple = mesh.face_tuple_from_vids(0, 1, 0);
    Eigen::Vector2d uv0 = pos.const_vector_attribute(tuple);
    Eigen::Vector2d uv1 = pos.const_vector_attribute(mesh.switch_vertex(tuple));
    Eigen::Vector2d uv2 = pos.const_vector_attribute(mesh.switch_vertex(mesh.switch_edge(tuple)));

    CHECK((uv0 - uv1).norm() - (uv1 - uv2).norm() < 1e-6);
    CHECK((uv0 - uv1).norm() - (uv0 - uv2).norm() < 1e-6);
    CHECK((uv1 - uv2).norm() - (uv0 - uv2).norm() < 1e-6);
}

// TEST_CASE("smoothing_Newton_Method_line_search")
// {
//     DEBUG_TriMesh mesh = ten_triangles_with_position(2);
//     OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy> op_settings;
//     op_settings.coordinate_handle =
//         mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
//     op_settings.smooth_boundary = false;
//     op_settings.second_order = true;
//     op_settings.line_search = true;
//     op_settings.energy = std::make_unique<function::AMIPS2D>(mesh,
//     op_settings.coordinate_handle); op_settings.initialize_invariants(mesh); Scheduler
//     scheduler(mesh); const auto& factory =
//         scheduler.add_operation_type<operations::tri_mesh::VertexSmoothUsingDifferentiableEnergy>(
//             "optimize_vertices",
//             std::move(op_settings));
//     Tuple tuple = mesh.face_tuple_from_vids(2, 4, 5);
//     spdlog::warn(
//         "Initial gradient: norm: {}",
//         factory.settings().energy->get_one_ring_gradient(tuple).norm());
//     while (factory.settings().energy->get_one_ring_gradient(tuple).norm() > 1e-10) {
//         scheduler.run_operation_on_all(PrimitiveType::Vertex, "optimize_vertices");
//         tuple = mesh.face_tuple_from_vids(2, 4, 5);
//     }
//     ConstAccessor<double> pos = mesh.create_const_accessor(op_settings.coordinate_handle);

//     Eigen::Vector2d uv0 = pos.const_vector_attribute(tuple);
//     Eigen::Vector2d uv1 = pos.const_vector_attribute(mesh.switch_vertex(tuple));
//     Eigen::Vector2d uv2 =
//     pos.const_vector_attribute(mesh.switch_vertex(mesh.switch_edge(tuple))); CHECK((uv0 -
//     uv1).norm() - (uv1 - uv2).norm() < 1e-6); CHECK((uv0 - uv1).norm() - (uv0 - uv2).norm() <
//     1e-6); CHECK((uv1 - uv2).norm() - (uv0 - uv2).norm() < 1e-6);
// }
