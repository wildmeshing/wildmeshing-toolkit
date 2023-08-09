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
#include "../tools/TriMesh_examples.hpp"

using json = nlohmann::json;
using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_isotropic_remeshing", "[components],[isotropic_remeshing]")
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

TEST_CASE("laplacian_smoothing", "[components],[isotropic_remeshing]")
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
