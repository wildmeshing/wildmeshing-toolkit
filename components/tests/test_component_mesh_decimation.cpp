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
        {"constrait_value", 1},
        {"target_len", 1.0},
        {"cell_constrait_tag_name", "tag"},
        {"pass_through", {"vertices"}}};

    CHECK_NOTHROW(o.get<MeshDecimationOptions>());
}

TEST_CASE("preprocess", "[.]")
{
    using namespace wmtk::components;

    auto mesh_in = wmtk::read_mesh(data_dir / "3d_images/sphere_regularized.hdf5");
    Mesh& mesh = static_cast<Mesh&>(*mesh_in);

    std::vector<wmtk::attribute::MeshAttributeHandle> pass_though;
    wmtk::attribute::MeshAttributeHandle vertex =
        mesh.get_attribute_handle<int64_t>("vertex_tag", PrimitiveType::Vertex);
    wmtk::attribute::MeshAttributeHandle edge =
        mesh.get_attribute_handle<int64_t>("edge_tag", PrimitiveType::Edge);
    wmtk::attribute::MeshAttributeHandle face =
        mesh.get_attribute_handle<int64_t>("face_tag", PrimitiveType::Triangle);
    pass_though.push_back(vertex);
    pass_though.push_back(edge);
    pass_though.push_back(face);
    internal::MeshDecimation MD(mesh, "tag", 1, 5, pass_though);
    MD.process();

    wmtk::io::ParaviewWriter
        writer(data_dir / "3d_images/out.hdf", "vertices", mesh, false, true, true, false);
    mesh.serialize(writer);
}