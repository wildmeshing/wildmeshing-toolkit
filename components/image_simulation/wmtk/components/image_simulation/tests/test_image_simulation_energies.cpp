#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <wmtk/components/image_simulation/ImageSimulationMeshTri.hpp>
#include <wmtk/components/image_simulation/read_image_msh.hpp>

namespace fs = std::filesystem;

using namespace wmtk;
using namespace components::image_simulation;

const fs::path data_dir = WMTK_DATA_DIR;

const std::string test_groups = "[image_simulation][2D][energies]";
#ifdef NDEBUG
const std::string test_release_only = "";
#else
const std::string test_release_only = "[.]";
#endif

std::shared_ptr<tri::ImageSimulationMeshTri> init_optimization_tests(Parameters& params)
{
    const fs::path model_path = data_dir / "models" / "lego_separated.msh";
    REQUIRE(fs::exists(model_path));

    const auto input_data = read_image_msh(model_path.string());

    params.dhat_rel = 3e-2;
    params.epsr = 1e-2;
    params.init(input_data.V_input.colwise().minCoeff(), input_data.V_input.colwise().maxCoeff());

    auto mesh = std::make_shared<tri::ImageSimulationMeshTri>(params, params.eps);
    mesh->init_from_image(input_data.V_input, input_data.T_input, input_data.T_input_tag);

    return mesh;
}

TEST_CASE("scale-invariance", test_groups)
{
    //
}

TEST_CASE("barrier-bfs", test_groups + test_release_only)
{
    using Tuple = TriMesh::Tuple;
    Parameters params;
    auto mesh_ptr = init_optimization_tests(params);
    tri::ImageSimulationMeshTri& mesh = *mesh_ptr;

    const auto& VA = mesh.m_vertex_attribute;

    for (const Tuple& t : mesh.get_vertices()) {
        const size_t vid = t.vid(mesh);
        if (vid < 8000) {
            continue;
        }
        if (!VA[vid].m_is_on_surface) {
            continue;
        }

        mesh.m_s_barrier = 1; // remove weight

        auto loc = mesh.get_barrier_energy(t, false);
        auto glob = mesh.get_barrier_energy(t, true);

        const Vector2d& x = VA[vid].m_pos;

        double v_loc = loc->value(x);
        VectorXd g_loc;
        loc->gradient(x, g_loc);
        MatrixXd h_loc;
        loc->hessian(x, h_loc);

        double v_glob = glob->value(x);
        VectorXd g_glob;
        glob->gradient(x, g_glob);
        MatrixXd h_glob;
        glob->hessian(x, h_glob);

        // logger()
        //     .info("v = {}: g_loc = {}, g_glob = {}", vid, g_loc.transpose(), g_glob.transpose());

        CHECK((g_loc - g_glob).squaredNorm() < 1e-10);
        CHECK((h_loc - h_glob).squaredNorm() < 1e-10);
    }
}