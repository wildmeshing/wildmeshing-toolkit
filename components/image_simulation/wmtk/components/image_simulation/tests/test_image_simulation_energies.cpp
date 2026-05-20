#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <wmtk/Types.hpp>
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

std::shared_ptr<tri::ImageSimulationMeshTri> init_optimization_tests(
    Parameters& params,
    double input_scale = 1.0)
{
    const fs::path model_path = data_dir / "models" / "lego_separated.msh";
    REQUIRE(fs::exists(model_path));

    auto input_data = read_image_msh(model_path.string());
    input_data.V_input *= input_scale;

    params.epsr = 1e-2;
    params.init(input_data.V_input.colwise().minCoeff(), input_data.V_input.colwise().maxCoeff());

    auto mesh = std::make_shared<tri::ImageSimulationMeshTri>(params, params.eps);
    mesh->init_from_image(
        input_data.V_input,
        input_data.T_input,
        input_data.T_input_tag,
        input_data.tag_names);

    return mesh;
}

void compare(const double a, const double b, const double tol = 1e-12)
{
    REQUIRE(((a != 0) && (b != 0)));
    const double rel = std::abs(a - b) / std::abs(a);
    // logger().info("a = {}, b = {}, rel = {}", a, b, rel);
    CHECK(rel < tol);
}

TEST_CASE("scale-invariance", test_groups + test_release_only)
{
    logger().set_level(spdlog::level::off);

    using Tuple = TriMesh::Tuple;

    struct Energies
    {
        double amips = -1, envelope = -1;
    };

    auto collect_energies = [](double scale) {
        Parameters params;
        auto mesh_ptr = init_optimization_tests(params, scale);
        tri::ImageSimulationMeshTri& mesh = *mesh_ptr;

        const auto& VA = mesh.m_vertex_attribute;

        std::vector<Energies> energies;

        for (const Tuple& t : mesh.get_vertices()) {
            const size_t vid = t.vid(mesh);
            if (vid < 9700) {
                continue;
            }
            if (!VA[vid].m_is_on_surface) {
                continue;
            }

            auto amips_energy = mesh.get_amips_energy(t);
            auto envelope_energy = mesh.get_envelope_energy(t);

            // find a position to test the envelope energy
            Tuple n;
            for (const Tuple& tt : mesh.get_one_ring_edges_for_vertex(vid)) {
                if (!VA[tt.vid(mesh)].m_is_on_surface) {
                    n = tt;
                    break;
                }
            }
            REQUIRE(n.is_valid(mesh));

            const Vector2d& x = VA[vid].m_pos;
            const Vector2d& y = VA[n.vid(mesh)].m_pos;

            Energies e;
            e.amips = amips_energy->value(x);
            e.envelope = envelope_energy->value(y);
            energies.push_back(e);
        }

        return energies;
    };


    const auto a = collect_energies(1);
    const auto b = collect_energies(1024);

    REQUIRE(a.size() == b.size());
    for (size_t i = 0; i < a.size(); ++i) {
        // compare all energies
        // logger().info("amips");
        compare(a[i].amips, b[i].amips);
        // logger().info("env");
        compare(a[i].envelope, b[i].envelope);
    }
}