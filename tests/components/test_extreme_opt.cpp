#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/operations/OperationFactory.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapseToMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplitAtMidpoint.hpp>
#include <wmtk/operations/tri_mesh/EdgeSwapValence.hpp>
#include <wmtk/operations/tri_mesh/VertexLaplacianSmooth.hpp>
#include <wmtk/operations/tri_mesh/VertexTangentialLaplacianSmooth.hpp>
#include <wmtk_components/input/input.hpp>

#include <wmtk_components/extreme_opt/internal/ExtremeOpt.hpp>
#include <wmtk_components/extreme_opt/internal/ExtremeOptOptions.hpp>

#include <wmtk_components/output/output.hpp>
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/TriMesh_examples.hpp"


#include <catch2/catch_test_macros.hpp>
#include <fstream>
#include <wmtk/Types.hpp>
#include <wmtk/function/SYMDIR.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapse.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplit.hpp>
#include "../tools/DEBUG_EdgeMesh.hpp"
#include "../tools/DEBUG_TriMesh.hpp"
#include "../tools/DEBUG_Tuple.hpp"
#include "../tools/EdgeMesh_examples.hpp"
#include "../tools/TriMesh_examples.hpp"

using json = nlohmann::json;
using namespace wmtk;
using namespace wmtk::tests;

const std::filesystem::path data_dir = WMTK_DATA_DIR;

using TM = TriMesh;
using TMOE = decltype(std::declval<DEBUG_TriMesh>().get_tmoe(
    wmtk::Tuple(),
    std::declval<Accessor<long>&>()));

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;

TEST_CASE("test_extreme_opt_io_cup", "[.]")
{
    using namespace operations;
    using namespace wmtk::components::internal;
    std::map<std::string, std::filesystem::path> files;

    ExtremeOptOptions options;
    {
        json test_extremeopt_settings;
        std::ifstream f("../json/test_extremeopt_settings.json");
        test_extremeopt_settings = json::parse(f);
        options = test_extremeopt_settings.get<ExtremeOptOptions>();
    }

    std::string mesh_name = options.mesh_name;

    {
        json input_seamed_mesh = {
            {"type", "input"},
            {"cell_dimension", 2},
            {"file", (data_dir / "extreme_opt_data_msh").string() + "/" + mesh_name + "_pos.msh"},
            {"name", "seamed_mesh"}};
        json input_cut_mesh = {
            {"type", "input"},
            {"cell_dimension", 2},
            {"file", (data_dir / "extreme_opt_data_msh").string() + "/" + mesh_name + "_tex.msh"},
            {"name", "cut_mesh"}};
        components::input(input_seamed_mesh, files);
        components::input(input_cut_mesh, files);
    }

    const std::filesystem::path seamed_mesh_file = files["seamed_mesh"];
    const std::filesystem::path cut_mesh_file = files["cut_mesh"];

    auto seamed_mesh_ptr = wmtk::read_mesh(seamed_mesh_file);
    tests::DEBUG_TriMesh& seamed_mesh = static_cast<tests::DEBUG_TriMesh&>(*seamed_mesh_ptr);
    std::shared_ptr<DEBUG_TriMesh> cut_mesh_ptr =
        std::static_pointer_cast<DEBUG_TriMesh>(wmtk::read_mesh(cut_mesh_file));
    auto child_map = multimesh::same_simplex_dimension_bijection(seamed_mesh, *cut_mesh_ptr);
    seamed_mesh.register_child_mesh(cut_mesh_ptr, child_map);
    seamed_mesh.multi_mesh_manager().check_map_valid(seamed_mesh);
}


TEST_CASE("test_simple_energy", "[.]")
{
    using namespace operations;
    using namespace wmtk::components::internal;
    std::map<std::string, std::filesystem::path> files;

    std::string mesh_name = "cube";
    {
        json input_cut_mesh = {
            {"type", "input"},
            {"cell_dimension", 2},
            {"file", (data_dir / "extreme_opt_data_msh").string() + "/" + mesh_name + "_tex.msh"},
            {"name", "cut_mesh"}};
        components::input(input_cut_mesh, files);
    }
    const std::filesystem::path cut_mesh_file = files["cut_mesh"];
    std::shared_ptr<DEBUG_TriMesh> parent_mesh_ptr =
        std::static_pointer_cast<DEBUG_TriMesh>(wmtk::read_mesh(cut_mesh_file));

    std::shared_ptr<DEBUG_TriMesh> child_mesh_ptr =
        std::static_pointer_cast<DEBUG_TriMesh>(wmtk::read_mesh(cut_mesh_file));

    auto child_map = multimesh::same_simplex_dimension_bijection(*parent_mesh_ptr, *child_mesh_ptr);
    parent_mesh_ptr->register_child_mesh(child_mesh_ptr, child_map);

    auto uv_handle =
        child_mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto ref_handle =
        parent_mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);


    wmtk::function::SYMDIR symdir_ref(
        *parent_mesh_ptr,
        *child_mesh_ptr,
        ref_handle,
        uv_handle,
        true);

    double energy_avg = symdir_ref.get_energy_avg();
    std::cout << energy_avg << std::endl;
    CHECK(abs(energy_avg - 4.0) < 1e-8);
}
