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

    double length_rel = options.length_rel;
    double length_abs = 0;
    {
        auto pos_handle =
            cut_mesh_ptr->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        auto pos = cut_mesh_ptr->create_const_accessor(pos_handle);

        Eigen::Vector2d p_max;
        p_max.setConstant(std::numeric_limits<double>::lowest());
        Eigen::Vector2d p_min;
        p_min.setConstant(std::numeric_limits<double>::max());

        for (const Tuple& v : cut_mesh_ptr->get_all(PrimitiveType::Vertex)) {
            const Eigen::Vector3d p = pos.const_vector_attribute(v);
            p_max[0] = std::max(p_max[0], p[0]);
            p_max[1] = std::max(p_max[1], p[1]);
            // p_max[2] = std::max(p_max[2], p[2]);
            p_min[0] = std::min(p_min[0], p[0]);
            p_min[1] = std::min(p_min[1], p[1]);
            // p_min[2] = std::min(p_min[2], p[2]);
        }
        const double diag_length = (p_max - p_min).norm();
        length_abs = diag_length * length_rel;
        std::cout << "length_abs: " << length_abs << std::endl;
    }
    bool preserve_childmesh_topology = true;
    bool preserve_childmesh_geometry = false;

    ExtremeOpt extreme_opt(
        mesh_name,
        seamed_mesh,
        length_abs,
        options.lock_boundary,
        preserve_childmesh_topology,
        preserve_childmesh_geometry,
        options.do_split,
        options.do_collapse,
        options.do_swap,
        options.do_smooth,
        options.debug_output);

    extreme_opt.remeshing(options.iterations);
    seamed_mesh.multi_mesh_manager().check_map_valid(seamed_mesh);


    // write final result to file
    {
        ParaviewWriter writer(
            "extreme_opt_" + mesh_name + "seamed_final",
            "vertices",
            seamed_mesh,
            true,
            true,
            true,
            false);
        seamed_mesh.serialize(writer);
        ParaviewWriter writer2(
            "extreme_opt_" + mesh_name + "cut_final",
            "vertices",
            *cut_mesh_ptr,
            true,
            true,
            true,
            false);
        cut_mesh_ptr->serialize(writer2);
    }
}
// TEST_CASE("test_extreme_opt_simple", "[multimesh][autogen][2D]")
// {
//     using namespace operations;

//     std::string path_to_data;
//     path_to_data.append(WMTK_DATA_DIR);
//     path_to_data.append("/extreme_opt_data/");
//     std::vector<std::string> models = {"elk", "helmet", "eight"};
//     for (auto model : models) {
//         std::cout << "test on model: " << model << std::endl;
//         std::string input_file = path_to_data + model + "_seamed.obj";

//         Eigen::MatrixXd V, uv, Vn;
//         RowVectors3l F, Fuv, Fn;
//         igl::readOBJ(input_file, V, uv, Vn, F, Fuv, Fn);

//         DEBUG_TriMesh parent;
//         parent.initialize(F);
//         REQUIRE(parent.is_connectivity_valid());
//         DEBUG_TriMesh child;
//         child.initialize(Fuv);
//         auto child_ptr = std::make_shared<DEBUG_TriMesh>(child);
//         REQUIRE(child_ptr->is_connectivity_valid());
//         std::vector<long> f_map(Fuv.rows());
//         for (long i = 0; i < long(f_map.size()); i++) {
//             f_map[i] = i;
//         }

//         MultiMeshManager::register_child_mesh(parent, child_ptr, f_map);
//         REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);

//         std::mt19937 rng(0);

//         std::cout << "test split" << std::endl;
//         for (auto i = 0; i < 200; i++) {
//             std::uniform_int_distribution<int> distribution(0, parent.capacity(PE) - 1);
//             long random_edge_id = distribution(rng);
//             std::cout << random_edge_id << std::endl;
//             if (parent.is_edge_deleted(random_edge_id)) {
//                 std::cout << "edge " << random_edge_id << " deleted" << std::endl;
//                 continue;
//             }
//             Tuple edge = parent.tuple_from_id(PE, random_edge_id);
//             if (parent.is_valid_slow(edge) == false) {
//                 std::cout << "edge " << random_edge_id << " invalid" << std::endl;
//                 continue;
//             }

//             OperationSettings<tri_mesh::EdgeSplit> op_settings;
//             op_settings.initialize_invariants(parent);
//             op_settings.split_boundary_edges = true;
//             tri_mesh::EdgeSplit op(parent, edge, op_settings);
//             const bool success = op();
//             REQUIRE(success == true);
//             REQUIRE(parent.is_connectivity_valid());
//             REQUIRE(child_ptr->is_connectivity_valid());
//             REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);
//         }

//         std::cout << "test collapse" << std::endl;
//         // test collapse
//         for (auto i = 0; i < 200; i++) {
//             std::uniform_int_distribution<int> distribution(0, parent.capacity(PE) - 1);
//             long random_edge_id = distribution(rng);
//             std::cout << random_edge_id << std::endl;

//             if (parent.is_edge_deleted(random_edge_id)) {
//                 std::cout << "edge " << random_edge_id << " deleted" << std::endl;
//                 continue;
//             }

//             Tuple edge = parent.tuple_from_id(PE, random_edge_id);
//             OperationSettings<tri_mesh::EdgeCollapse> op_settings;
//             op_settings.initialize_invariants(parent);
//             tri_mesh::EdgeCollapse op(parent, edge, op_settings);
//             op();
//             REQUIRE(parent.is_connectivity_valid());
//             REQUIRE(child_ptr->is_connectivity_valid());
//             REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);
//         }
//     }
// }

// TEST_CASE("test_extreme_opt_all", "[multimesh][autogen][2D][.]")
// {
//     using namespace operations;

//     std::string path_to_data;
//     path_to_data.append(WMTK_DATA_DIR);
//     path_to_data.append("/extreme_opt_data/");
//     std::vector<std::string> models = {
//         "chair",
//         "elephant",
//         "neptune0",
//         "rocker_arm",
//         "botijo",
//         "cup",
//         "elk",
//         "helmet",
//         "femur",
//         "holes3",
//         "sculpt",
//         "bumpy_torus",
//         "dancer2",
//         "fertility_tri",
//         "thai_statue",
//         "camel",
//         "eight",
//         "bozbezbozzel100K",
//         "carter100K",
//         "chair100K",
//         "dancer_25k",
//         "dancing_children100K",
//         "dragonstand_recon100K",
//         "genus3",
//         "kitten100K",
//         "knot100K",
//         "master_cylinder100K",
//         "rolling_stage100K",
//         "wrench50K",
//         "pulley100K",
//         "pegaso"};
//     for (auto model : models) {
//         std::cout << "test on model: " << model << std::endl;
//         std::string input_file = path_to_data + model + "_seamed.obj";

//         Eigen::MatrixXd V, uv, Vn;
//         RowVectors3l F, Fuv, Fn;
//         igl::readOBJ(input_file, V, uv, Vn, F, Fuv, Fn);

//         DEBUG_TriMesh parent;
//         parent.initialize(F);
//         REQUIRE(parent.is_connectivity_valid());
//         DEBUG_TriMesh child;
//         child.initialize(Fuv);
//         auto child_ptr = std::make_shared<DEBUG_TriMesh>(child);
//         REQUIRE(child_ptr->is_connectivity_valid());
//         std::vector<long> f_map(Fuv.rows());
//         for (long i = 0; i < long(f_map.size()); i++) {
//             f_map[i] = i;
//         }

//         MultiMeshManager::register_child_mesh(parent, child_ptr, f_map);
//         REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);

//         std::mt19937 rng(0);

//         std::cout << "test split" << std::endl;
//         for (auto i = 0; i < 200; i++) {
//             std::uniform_int_distribution<int> distribution(0, parent.capacity(PE) - 1);
//             long random_edge_id = distribution(rng);
//             std::cout << random_edge_id << std::endl;
//             if (parent.is_edge_deleted(random_edge_id)) {
//                 std::cout << "edge " << random_edge_id << " deleted" << std::endl;
//                 continue;
//             }
//             Tuple edge = parent.tuple_from_id(PE, random_edge_id);
//             if (parent.is_valid_slow(edge) == false) {
//                 std::cout << "edge " << random_edge_id << " invalid" << std::endl;
//                 continue;
//             }

//             OperationSettings<tri_mesh::EdgeSplit> op_settings;
//             op_settings.initialize_invariants(parent);
//             op_settings.split_boundary_edges = true;
//             tri_mesh::EdgeSplit op(parent, edge, op_settings);
//             const bool success = op();
//             REQUIRE(success == true);
//             REQUIRE(parent.is_connectivity_valid());
//             REQUIRE(child_ptr->is_connectivity_valid());
//             REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);
//         }

//         std::cout << "test collapse" << std::endl;
//         // test collapse
//         for (auto i = 0; i < 200; i++) {
//             std::uniform_int_distribution<int> distribution(0, parent.capacity(PE) - 1);
//             long random_edge_id = distribution(rng);
//             std::cout << random_edge_id << std::endl;

//             if (parent.is_edge_deleted(random_edge_id)) {
//                 std::cout << "edge " << random_edge_id << " deleted" << std::endl;
//                 continue;
//             }

//             Tuple edge = parent.tuple_from_id(PE, random_edge_id);
//             OperationSettings<tri_mesh::EdgeCollapse> op_settings;
//             op_settings.initialize_invariants(parent);
//             tri_mesh::EdgeCollapse op(parent, edge, op_settings);
//             op();
//             REQUIRE(parent.is_connectivity_valid());
//             REQUIRE(child_ptr->is_connectivity_valid());
//             REQUIRE(parent.multi_mesh_manager.is_map_valid(parent) == true);
//         }
//     }
// }
