#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/Stopwatch.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/shortestedge_collapse/shortestedge_collapse.hpp>
#include <wmtk/components/tetwild_simplification/tetwild_simplification.hpp>
#include <wmtk/components/utils/get_attributes.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

using namespace wmtk;
using namespace components;
namespace fs = std::filesystem;

using wmtk::components::utils::resolve_paths;

nlohmann::json test_run(const fs::path& input_file, const bool run_tetwild_simplification)
{
    std::shared_ptr<Mesh> mesh_in = input(input_file);

    logger().info(
        "Input {} #faces = {}",
        run_tetwild_simplification ? "tws" : "sec",
        mesh_in->get_all(PrimitiveType::Triangle).size());

    attribute::MeshAttributeHandle pos_handle =
        mesh_in->get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    if (run_tetwild_simplification) {
        // tetwild_simplification
        nlohmann::json stats;
        {
            wmtk::utils::StopWatch sw("tetwild_simplification");
            std::tie(mesh_in, stats) =
                tetwild_simplification(static_cast<TriMesh&>(*mesh_in), "vertices", 1000);
        }
    } else {
        // shortestedge_collapse
        wmtk::utils::StopWatch sw("shortestedge_collapse");
        shortestedge_collapse(static_cast<TriMesh&>(*mesh_in), pos_handle, 1000);
    }
    output(*mesh_in, run_tetwild_simplification ? "tws_out" : "sec_out", pos_handle);

    logger().info(
        "Output {} #faces = {}",
        run_tetwild_simplification ? "tws" : "sec",
        mesh_in->get_all(PrimitiveType::Triangle).size());

    nlohmann::json out_json;
    out_json["stats"]["vertices"] = mesh_in->get_all(PrimitiveType::Vertex).size();
    out_json["stats"]["edges"] = mesh_in->get_all(PrimitiveType::Edge).size();
    out_json["stats"]["triangles"] = mesh_in->get_all(PrimitiveType::Triangle).size();
    out_json["stats"]["tets"] = mesh_in->get_all(PrimitiveType::Tetrahedron).size();

    return out_json;
}

int main(int argc, char* argv[])
{
    const fs::path input_file = "100071_sf.msh"; // 11,040 faces
    //const fs::path input_file = "blub.msh"; // 14,208 faces
    //const fs::path input_file = "Octocat.msh"; // 37,884 faces
    //const fs::path input_file = "bunny.msh"; // 69,451 faces
    //const fs::path input_file = "max-planck.msh"; // 99,991 faces

    nlohmann::json out_json;

    // shortestedge_collapse
    out_json["sec"] = test_run(input_file, false);

    // tetwild_simplification
    out_json["tws"] = test_run(input_file, true);


    const std::string report = "report.json";
    if (!report.empty()) {
        // out_json["input"] = j;

        std::ofstream ofs(report);
        ofs << std::setw(4) << out_json;
    }


    return 0;
}
