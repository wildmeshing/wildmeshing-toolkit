#include <catch2/catch_test_macros.hpp>
#include <nlohmann/json.hpp>
#include <wmtk/io/Cache.hpp>
#include "wmtk/TetMesh.hpp"
#include "wmtk/io/HDF5Reader.hpp"
#include "wmtk/utils/Logger.hpp"
#include "wmtk_components/feb_generator/wmtk/components/feb_generator/feb_generator.hpp"
#include "wmtk_components/feb_generator/wmtk/components/feb_generator/internal/FEBGenerator.hpp"

using json = nlohmann::json;

TEST_CASE("json_test_temp", "[.]")
{
    wmtk::io::Cache cache("wmtk_cache", ".");
    wmtk::HDF5Reader reader;
    auto mesh_in = reader.read(
        "/home/zhouyuan/workplace/slicer_extract/wildmeshing-toolkit/data/tag_mesh.out.hdf5");

    wmtk::TetMesh& m = static_cast<wmtk::TetMesh&>(*mesh_in);

    json res = wmtk::components::internal::read_json_settings(
        "/home/zhouyuan/workplace/slicer_extract/wildmeshing-toolkit/data/Selections_Info.json");

    std::string output = "";
    wmtk::components::internal::generate_feb_files(m, res, output);
}
//