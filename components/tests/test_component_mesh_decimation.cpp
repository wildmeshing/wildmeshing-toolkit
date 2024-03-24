#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <tools/DEBUG_TetMesh.hpp>
#include <tools/DEBUG_TriMesh.hpp>
#include <tools/TetMesh_examples.hpp>
#include <tools/TriMesh_examples.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/components/mesh_decimation/internal/MeshDecimation.hpp>
#include <wmtk/components/mesh_decimation/internal/MeshDecimationOptions.hpp>
#include <wmtk/components/mesh_decimation/mesh_decimation.hpp>
#include <wmtk/function/simplex/AMIPS.hpp>
#include <wmtk/invariants/TodoInvariant.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/io/ParaviewWriter.hpp>
#include <wmtk/operations/EdgeCollapse.hpp>
#include <wmtk/operations/attribute_new/CollapseNewAttributeStrategy.hpp>
#include <wmtk/simplex/link.hpp>
#include <wmtk/utils/mesh_utils.hpp>

using json = nlohmann::json;
using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("component_mesh_decimation_options", "[components][mesh_decimation]")
{
    using namespace components::internal;

    json o = {
        {"input", "input_mesh"},
        {"output", "output_mesh"},
        {"target_len", 1.0},
        {"cell_constraint_tag_name", "tag"},
        {"attributes", {"vertices", "tag"}},
        {"pass_through", {"dummy"}}};

    CHECK_NOTHROW(o.get<MeshDecimationOptions>());
}

TEST_CASE("component_mesh_decimation", "[components][2D][3D]")
{
    using namespace wmtk::components;
    wmtk::io::Cache cache("wmtk_cache", ".");

    SECTION("3D")
    {
        auto mesh_in = wmtk::read_mesh(data_dir / "unit_test/meshes/sphere_regularized.hdf5");
        Mesh& mesh = *mesh_in;

        std::vector<wmtk::attribute::MeshAttributeHandle> keep;
        wmtk::attribute::MeshAttributeHandle constrait_cell_tag_handle =
            mesh.get_attribute_handle<int64_t>("tag", mesh.top_simplex_type());
        wmtk::attribute::MeshAttributeHandle pos_handle =
            mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        keep.emplace_back(constrait_cell_tag_handle);
        keep.emplace_back(pos_handle);
        mesh.clear_attributes(keep);

        std::vector<wmtk::attribute::MeshAttributeHandle> pass_though;
        internal::MeshDecimation md(mesh, constrait_cell_tag_handle, 5, pass_though);
        md.process();

        cache.write_mesh(mesh, "out3d");
    }

    SECTION("2D")
    {
        auto mesh_in = wmtk::read_mesh(data_dir / "2d/ellipse_layer/ellipse_01_substructure.hdf5");
        Mesh& mesh = *mesh_in;

        std::vector<wmtk::attribute::MeshAttributeHandle> keep;
        wmtk::attribute::MeshAttributeHandle constrait_cell_tag_handle =
            mesh.get_attribute_handle<int64_t>("tag", mesh.top_simplex_type());
        wmtk::attribute::MeshAttributeHandle pos_handle =
            mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        keep.emplace_back(constrait_cell_tag_handle);
        keep.emplace_back(pos_handle);
        mesh.clear_attributes(keep);

        std::vector<wmtk::attribute::MeshAttributeHandle> pass_though;
        internal::MeshDecimation md(mesh, constrait_cell_tag_handle, 5, pass_though);
        md.process();

        cache.write_mesh(mesh, "out2d");
    }
}