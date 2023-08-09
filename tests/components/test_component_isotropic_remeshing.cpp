#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/operations/OperationFactory.hpp>
#include <wmtk/operations/TriMeshCollapseEdgeOperation.hpp>
#include <wmtk/operations/TriMeshSplitEdgeOperation.hpp>
#include <wmtk/operations/TriMeshVertexSmoothOperation.hpp>
#include <wmtk_components/input/input.hpp>
#include <wmtk_components/isotropic_remeshing/internal/IsotropicRemeshing.hpp>
#include <wmtk_components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk_components/output/output.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"

using json = nlohmann::json;
using namespace wmtk;
using namespace wmtk::tests;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_isotropic_remeshing", "[components][isotropic_remeshing][2D][.]")
{
    std::map<std::string, std::filesystem::path> files;

    json input_component_json = {
        {"type", "input"},
        {"name", "input_mesh"},
        {"file", data_dir / "bunny.off"}};


    wmtk::components::input(input_component_json, files);


    SECTION("should pass")
    {
        json mesh_isotropic_remeshing_json = {
            {"type", "isotropic_remeshing"},
            {"input", "input_mesh"},
            {"output", "output_mesh"},
            {"length_abs", 0.003},
            {"length_rel", -1},
            {"iterations", 3},
            {"lock_boundary", true}};
        CHECK_NOTHROW(wmtk::components::isotropic_remeshing(mesh_isotropic_remeshing_json, files));

        {
            json component_json = {
                {"type", "output"},
                {"input", "output_mesh"},
                {"file", "bunny_isotropic_remeshing"}};

            CHECK_NOTHROW(wmtk::components::output(component_json, files));
        }
    }
}

TEST_CASE("smoothing_bunny", "[components][isotropic_remeshing][2D]")
{
    std::map<std::string, std::filesystem::path> files;

    // input
    {
        json input_component_json = {
            {"type", "input"},
            {"name", "input_mesh"},
            {"file", data_dir / "bunny.off"}};
        wmtk::components::input(input_component_json, files);
    }

    wmtk::TriMesh mesh;
    {
        const std::filesystem::path& file = files["input_mesh"];
        wmtk::MeshReader reader(file);
        reader.read(mesh);
    }

    TriMeshVertexSmoothOperation::Settings op_settings;
    op_settings.position = mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);

    Scheduler scheduler(mesh);
    scheduler
        .add_operation_type<TriMeshVertexSmoothOperation, TriMeshVertexSmoothOperation::Settings>(
            "vertex_smooth",
            op_settings);

    for (int i = 0; i < 3; ++i) {
        scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_smooth");
    }

    // output
    {
        ParaviewWriter writer("bunny_smooth", "position", mesh, true, true, true, false);
        mesh.serialize(writer);
    }
}

TEST_CASE("smoothing_simple_examples", "[components][isotropic_remeshing][2D]")
{
    SECTION("hex_plus_two")
    {
        DEBUG_TriMesh mesh = wmtk::tests::hex_plus_two_with_position();

        TriMeshVertexSmoothOperation::Settings op_settings;
        op_settings.position = mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);

        // offset interior vertex
        auto pos = mesh.create_accessor(op_settings.position);
        const Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
        pos.vector_attribute(v4) = Eigen::Vector3d{0.6, 0.9, 0};

        Scheduler scheduler(mesh);
        scheduler.add_operation_type<
            TriMeshVertexSmoothOperation,
            TriMeshVertexSmoothOperation::Settings>("vertex_smooth", op_settings);

        scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_smooth");

        Eigen::Vector3d after_smooth = pos.vector_attribute(v4);
        CHECK((after_smooth - Eigen::Vector3d{1, 0, 0}).squaredNorm() == 0);
    }

    SECTION("edge_region")
    {
        DEBUG_TriMesh mesh = wmtk::tests::edge_region_with_position();

        TriMeshVertexSmoothOperation::Settings op_settings;
        op_settings.position = mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);

        // offset interior vertex
        auto pos = mesh.create_accessor(op_settings.position);
        const Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
        const Tuple v5 = mesh.tuple_from_id(PrimitiveType::Vertex, 5);
        pos.vector_attribute(v4) = Eigen::Vector3d{0.6, 0.9, 0};
        pos.vector_attribute(v5) = Eigen::Vector3d{1.4, -0.9, 0};

        Scheduler scheduler(mesh);
        scheduler.add_operation_type<
            TriMeshVertexSmoothOperation,
            TriMeshVertexSmoothOperation::Settings>("vertex_smooth", op_settings);

        for (size_t i = 0; i < 10; ++i) {
            scheduler.run_operation_on_all(PrimitiveType::Vertex, "vertex_smooth");
            Eigen::Vector3d p4_after_smooth = pos.vector_attribute(v4);
        }

        Eigen::Vector3d p4_after_smooth = pos.vector_attribute(v4);
        CHECK((p4_after_smooth - Eigen::Vector3d{1, 0, 0}).squaredNorm() < 1e-10);

        Eigen::Vector3d p5_after_smooth = pos.vector_attribute(v5);
        CHECK((p5_after_smooth - Eigen::Vector3d{2, 0, 0}).squaredNorm() < 1e-10);
    }
}

TEST_CASE("split_long_edges", "[components][isotropic_remeshing][split][2D][.]")
{
    // This test does not fully work yet

    DEBUG_TriMesh mesh = wmtk::tests::edge_region_with_position();

    TriMeshSplitEdgeOperation::Settings op_settings;
    op_settings.position = mesh.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    op_settings.min_squared_length = 1.1;

    auto pos = mesh.create_accessor(op_settings.position);
    const Tuple v4 = mesh.tuple_from_id(PrimitiveType::Vertex, 4);
    const Tuple v5 = mesh.tuple_from_id(PrimitiveType::Vertex, 5);
    // reposition interior vertices
    pos.vector_attribute(v4) = Eigen::Vector3d{0.6, 0.9, 0};
    pos.vector_attribute(v5) = Eigen::Vector3d{1.4, -0.9, 0};

    Scheduler scheduler(mesh);
    scheduler.add_operation_type<TriMeshSplitEdgeOperation, TriMeshSplitEdgeOperation::Settings>(
        "edge_split",
        op_settings);

    scheduler.run_operation_on_all(PrimitiveType::Edge, "edge_split");

    REQUIRE(mesh.get_all(PrimitiveType::Vertex).size() == 13);
}