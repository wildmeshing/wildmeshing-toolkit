// The tagged route's wall (TetWildMesh::init_from_tagged_mesh()): the boundary faces, untracked,
// each in a plane of the mesh's own bounding box, held by the box rule. A mesh with a tracked
// boundary face is refused: a filtered tetwild output is the case, its boundary its tracked faces.

#include <wmtk/components/tetwild/TetWildMesh.h>
#include <wmtk/components/tetwild/read_tagged_mesh.hpp>
#include <wmtk/components/tetwild/tetwild.hpp>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/io/tagged_mesh_data.hpp>

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

// Cases too slow for the Debug CI jobs, Windows Debug above all: Release builds only.
#ifdef NDEBUG
static const std::string release_only = "";
#else
static const std::string release_only = "[.]";
#endif

using namespace wmtk;
using namespace wmtk::components::tetwild;

namespace {

/// The unit cube's boundary, its triangles facing out.
MeshData unit_cube()
{
    MeshData data;
    data.V.resize(8, 3);
    data.V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1;
    auto& F = data.cells[CellType::Triangle].connectivity;
    F.resize(12, 3);
    F << 0, 3, 2, 0, 2, 1, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2, 0, 4, 7, 0, 7, 3,
        1, 2, 6, 1, 6, 5;
    return data;
}

/// A TetWildMesh initialized from `tagged` as the tagged route initializes it, around the
/// tagged faces as received.
std::unique_ptr<TetWildMesh> init_from(const MeshData& tagged, Parameters& params)
{
    const TaggedMesh in = read_tagged_mesh(io::tags_only(tagged), "tagged");
    Vector3d lo = in.V[0], hi = in.V[0];
    for (const Vector3d& p : in.V) {
        lo = lo.cwiseMin(p);
        hi = hi.cwiseMax(p);
    }
    params.epsr = 1e-3;
    params.init(lo, hi);
    std::vector<Eigen::Vector3i> F;
    for (const auto& f : in.F) F.emplace_back(int(f[0]), int(f[1]), int(f[2]));
    auto envelope = std::make_shared<Envelope>(true);
    envelope->init(in.V, F, params.eps);
    auto mesh = std::make_unique<TetWildMesh>(params, envelope, 1);
    mesh->m_input_names = in.face_names;
    mesh->init_from_tagged_mesh(in);
    return mesh;
}

} // namespace

TEST_CASE("tagged-route-refuses-a-tracked-boundary", "[tetwild][tagged]" + release_only)
{
    // The cube meshed and filtered to its inside: every boundary face is tracked, in a plane of
    // the box. Refused, naming the count, on initialization and through the driver.
    const auto filtered = wmtk::components::tetwild::tetwild(
        {{"input", {"a"}},
         {"input_names", {"a"}},
         {"simplify", false},
         {"filter", "tracked"},
         {"max_iterations", 0},
         {"num_threads", 1}},
        {{"a", unit_cube()}});
    const MeshData& mesh_in = filtered.first.at("mesh");
    Parameters params;
    REQUIRE_THROWS_WITH(
        init_from(mesh_in, params),
        Catch::Matchers::ContainsSubstring("tracked faces of the tagged mesh lie on its boundary"));
    REQUIRE_THROWS_WITH(
        wmtk::components::tetwild::tetwild(
            {{"input", "m"}, {"max_iterations", 3}, {"num_threads", 1}},
            {{"m", mesh_in}}),
        Catch::Matchers::ContainsSubstring("filter \"none\""));
}

TEST_CASE("tagged-route-untracked-boundary-is-the-wall", "[tetwild][tagged]" + release_only)
{
    // The same cube without a filter: the box around it is the wall, every untracked boundary
    // face flagged with its plane and its vertices with theirs; the tracked faces, inside, none.
    const auto boxed = wmtk::components::tetwild::tetwild(
        {{"input", {"a"}},
         {"input_names", {"a"}},
         {"simplify", false},
         {"max_iterations", 0},
         {"num_threads", 1}},
        {{"a", unit_cube()}});
    Parameters params;
    const auto mesh = init_from(boxed.first.at("mesh"), params);
    size_t n_wall = 0;
    for (const auto& f : mesh->get_faces()) {
        const auto& attr = mesh->m_face_attribute[f.fid(*mesh)];
        if (attr.m_is_surface_fs) {
            CHECK(attr.m_is_bbox_fs == -1);
            continue;
        }
        if (f.switch_tetrahedron(*mesh)) continue;
        ++n_wall;
        REQUIRE(attr.m_is_bbox_fs >= 0);
        for (const size_t vid : mesh->get_face_vids(f)) {
            const auto& planes = mesh->m_vertex_attribute[vid].on_bbox_faces;
            CHECK(std::count(planes.begin(), planes.end(), attr.m_is_bbox_fs) == 1);
        }
    }
    CHECK(n_wall > 0);
}
