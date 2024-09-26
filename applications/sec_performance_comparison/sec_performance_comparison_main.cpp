#include <filesystem>
#include <fstream>
#include <nlohmann/json.hpp>

#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/attribute/MeshAttributeHandle.hpp>
#include <wmtk/utils/Logger.hpp>

#include <wmtk/components/input/input.hpp>
#include <wmtk/components/output/output.hpp>
#include <wmtk/components/shortestedge_collapse/shortestedge_collapse.hpp>
#include <wmtk/components/tetwild_simplification/tetwild_simplification.hpp>
#include <wmtk/components/utils/get_attributes.hpp>
#include <wmtk/components/utils/resolve_path.hpp>

using namespace wmtk;
namespace fs = std::filesystem;

using wmtk::components::utils::resolve_paths;

int main(int argc, char* argv[])
{
    const fs::path input_file = "100071_sf.msh"; // 11,040 faces
    //const fs::path input_file = "blub.msh"; // 14,208 faces
    //const fs::path input_file = "Octocat.msh"; // 37,884 faces
    //const fs::path input_file = "bunny.msh"; // 69,451 faces
    //const fs::path input_file = "max-planck.msh"; // 99,991 faces

    std::shared_ptr<Mesh> mesh_in = wmtk::components::input(input_file);
    TriMesh& mesh = static_cast<TriMesh&>(*mesh_in);

    attribute::MeshAttributeHandle pos_handle =
        mesh.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);

    // shortestedge_collapse
    {
        components::shortestedge_collapse(mesh, pos_handle, 10);
    }

    // tetwild_simplification
    {
        //
    }

    wmtk::components::output(mesh, "sec_performance_comparison_out", pos_handle);

    const std::string report = "report.json";
    if (!report.empty()) {
        nlohmann::json out_json;
        out_json["stats"]["vertices"] = mesh.get_all(PrimitiveType::Vertex).size();
        out_json["stats"]["edges"] = mesh.get_all(PrimitiveType::Edge).size();
        out_json["stats"]["triangles"] = mesh.get_all(PrimitiveType::Triangle).size();
        out_json["stats"]["tets"] = mesh.get_all(PrimitiveType::Tetrahedron).size();

        // out_json["input"] = j;

        std::ofstream ofs(report);
        ofs << std::setw(4) << out_json;
    }


    return 0;
}
