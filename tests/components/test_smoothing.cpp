#pragma once
#include <catch2/catch_test_macros.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/function/AMIPS.hpp>
#include <wmtk/operations/tri_mesh/VertexSmoothNewtonMethodWithLineSearch.hpp>
#include <wmtk/operations/tri_mesh/VertexSmoothUsingDifferentiableEnergy.hpp>
#include "../tools/TriMesh_examples.hpp"
using namespace wmtk;
using namespace wmtk::tests;
using namespace wmtk::operations;

TEST_CASE("smoothing_using_differentiable_energy")
{
    TriMesh mesh = ten_triangles_with_position(2);
    OperationSettings<tri_mesh::VertexSmoothUsingDifferentiableEnergy> op_settings;
    op_settings.uv_position = mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    op_settings.smooth_boundary = false;
    op_settings.second_order = true;
    op_settings.line_search = false;
    op_settings.energy = std::make_unique<function::AMIPS_2D>(
        mesh,
        mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex));
    Scheduler scheduler(mesh);
    scheduler.add_operation_type<tri_mesh::VertexSmoothUsingDifferentiableEnergy>(
        "tri_mesh_smooth_vertex_newton_method",
        op_settings);
    scheduler.run_operation_on_all(PrimitiveType::Vertex, "tri_mesh_smooth_vertex_newton_method");
}