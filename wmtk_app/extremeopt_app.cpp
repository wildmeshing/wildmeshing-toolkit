#include <jse/jse.h>
#include <CLI/CLI.hpp>
#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>
#include <wmtk/Scheduler.hpp>
#include <wmtk/SimplicialComplex.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/Types.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/multimesh/same_simplex_dimension_bijection.hpp>
#include <wmtk/multimesh/same_simplex_dimension_surjection.hpp>
#include <wmtk/multimesh/utils/extract_child_mesh_from_tag.hpp>
#include <wmtk/multimesh/utils/tuple_map_attribute_io.hpp>
#include <wmtk/operations/OperationFactory.hpp>
#include <wmtk/operations/tri_mesh/EdgeCollapse.hpp>
#include <wmtk/operations/tri_mesh/EdgeSplit.hpp>
#include <wmtk/operations/tri_mesh/VertexLaplacianSmooth.hpp>
#include <wmtk/operations/tri_mesh/VertexTangentialLaplacianSmooth.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk_components/extreme_opt/internal/ExtremeOpt.hpp>
#include <wmtk_components/extreme_opt/internal/ExtremeOptOptions.hpp>
#include <wmtk_components/input/input.hpp>
#include <wmtk_components/isotropic_remeshing/isotropic_remeshing.hpp>
#include <wmtk_components/mesh_info/mesh_info.hpp>
#include <wmtk_components/output/output.hpp>

using json = nlohmann::json;
using namespace wmtk;
using namespace wmtk::tests;
const std::filesystem::path data_dir = WMTK_DATA_DIR;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;
int main(int argc, char** argv)
{
    using path = std::filesystem::path;
    using namespace operations;
    using namespace wmtk::components::internal;


    CLI::App app{argv[0]};
    path json_input_file;
    app.add_option("-j, --json", json_input_file, "json specification file")->required(true);


    CLI11_PARSE(app, argc, argv);
    if (!std::filesystem::exists(json_input_file)) {
        wmtk::logger().critical("File `{}` does not exist.", json_input_file);
        exit(-1);
    }

    std::map<std::string, std::filesystem::path> files;

    ExtremeOptOptions options;
    {
        json test_extremeopt_settings;
        std::ifstream f(json_input_file);
        test_extremeopt_settings = json::parse(f);
        options = test_extremeopt_settings.get<ExtremeOptOptions>();
    }

    std::string mesh_name = options.mesh_name;
    {
        json input_seamed_mesh = {
            {"type", "input"},
            {"cell_dimension", 2},
            {"file", (data_dir / "extreme_opt_data_msh").string() + "/" + mesh_name + "_pos.msh"},
            {"name", "seamed_mesh" + options.mesh_name}};
        json input_cut_mesh = {
            {"type", "input"},
            {"cell_dimension", 2},
            {"file", (data_dir / "extreme_opt_data_msh").string() + "/" + mesh_name + "_tex.msh"},
            {"name", "cut_mesh" + options.mesh_name}};
        components::input(input_seamed_mesh, files);
        components::input(input_cut_mesh, files);
    }

    const std::filesystem::path seamed_mesh_file = files["seamed_mesh" + options.mesh_name];
    const std::filesystem::path cut_mesh_file = files["cut_mesh" + options.mesh_name];

    auto seamed_mesh_ptr = wmtk::read_mesh(seamed_mesh_file);
    TriMesh& seamed_mesh = static_cast<TriMesh&>(*seamed_mesh_ptr);
    std::shared_ptr<TriMesh> cut_mesh_ptr =
        std::static_pointer_cast<TriMesh>(wmtk::read_mesh(cut_mesh_file));
    auto child_map = multimesh::same_simplex_dimension_bijection(seamed_mesh, *cut_mesh_ptr);
    seamed_mesh.register_child_mesh(cut_mesh_ptr, child_map);

    // for testing
    // {
    //     seamed_mesh.register_attribute<char>("freeze", PV, 1, false, 0);
    // }


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

    ExtremeOpt extreme_opt(
        mesh_name,
        seamed_mesh,
        length_abs,
        options.lock_boundary,
        options.do_split,
        options.do_collapse,
        options.collapse_optimize_E_max,
        options.do_swap,
        options.swap_optimize_E_max,
        options.do_smooth,
        options.debug_output);

    extreme_opt.remeshing(options.iterations);

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


    return 0;
}
