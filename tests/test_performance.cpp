#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/MeshReader.hpp>

using json = nlohmann::json;

using namespace wmtk;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

TEST_CASE("Read_only", "[performance]")
{
    const std::filesystem::path meshfile =
        data_dir / "adaptive_tessellation_test" / "after_smooth_uv.msh";

    auto mesh_in = wmtk::read_mesh(meshfile, true);
    Mesh& m = *mesh_in;

    auto pos_handle = m.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pos_acc = m.create_accessor<double>(pos_handle);

    double sum = 0;

    const auto vertices = m.get_all(PrimitiveType::Vertex);
    for (int i = 0; i < 10000; ++i) {
        for (const Tuple& t : vertices) {
            sum += pos_acc.const_vector_attribute(t)[0];
        }
        for (const Tuple& t : vertices) {
            sum += pos_acc.const_vector_attribute(t)[1];
        }
    }
    std::cout << "sum = " << sum << std::endl;
}