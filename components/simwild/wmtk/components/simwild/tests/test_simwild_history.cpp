#include <catch2/catch_test_macros.hpp>

#include <wmtk/components/simwild/read_image_msh.hpp>
#include <wmtk/components/simwild/simwild.hpp>
#include <wmtk/components/surface_map/SurfaceMap.hpp>
#include <wmtk/history/SurfaceHistory.hpp>

#include <algorithm>
#include <filesystem>
#include <map>
#include <set>

namespace fs = std::filesystem;
using namespace wmtk;
using namespace wmtk::components;

namespace {

/// A face by its corner positions, sorted: the same in the history and in a mesh file.
using Key = std::array<std::array<double, 3>, 3>;
/// The tag sets of the tets on either side of an interface face, the smaller first.
using Label = std::pair<std::set<int64_t>, std::set<int64_t>>;

Key key_of(const std::array<Eigen::Vector3d, 3>& p)
{
    Key k;
    for (int i = 0; i < 3; ++i) k[i] = {{p[i][0], p[i][1], p[i][2]}};
    std::sort(k.begin(), k.end());
    return k;
}

/// The interfaces of a tagged tet mesh: every face between two tets whose tag sets differ.
std::map<Key, Label> interfaces(const simwild::InputData& d)
{
    std::vector<std::set<int64_t>> tags(d.T_input.rows());
    for (Eigen::Index t = 0; t < d.T_input_tag.rows(); ++t) {
        for (Eigen::Index j = 0; j < d.T_input_tag.cols(); ++j) {
            if (d.T_input_tag.coeff(t, j) != 0) tags[t].insert(j);
        }
    }
    std::map<std::array<int, 3>, std::vector<Eigen::Index>> face_tets;
    for (Eigen::Index t = 0; t < d.T_input.rows(); ++t) {
        for (int skip = 0; skip < 4; ++skip) {
            std::array<int, 3> f;
            for (int k = 0, n = 0; k < 4; ++k) {
                if (k != skip) f[n++] = d.T_input(t, k);
            }
            std::sort(f.begin(), f.end());
            face_tets[f].push_back(t);
        }
    }
    std::map<Key, Label> out;
    for (const auto& [f, ts] : face_tets) {
        if (ts.size() != 2 || tags[ts[0]] == tags[ts[1]]) continue;
        std::array<Eigen::Vector3d, 3> p;
        for (int k = 0; k < 3; ++k) p[k] = d.V_input.row(f[k]).transpose();
        out[key_of(p)] = std::minmax(tags[ts[0]], tags[ts[1]]);
    }
    return out;
}

/// The label of every live face of a complex, looked up in `faces`; -1 entries count the misses.
std::vector<const Label*> labels_of(
    const std::vector<Eigen::Vector3d>& V,
    const std::vector<std::array<uint64_t, 3>>& F,
    const std::vector<uint8_t>& alive,
    const std::map<Key, Label>& faces,
    size_t& found)
{
    std::vector<const Label*> out(F.size(), nullptr);
    found = 0;
    for (size_t f = 0; f < F.size(); ++f) {
        if (!alive[f]) continue;
        const auto it = faces.find(key_of({{V[F[f][0]], V[F[f][1]], V[F[f][2]]}}));
        if (it == faces.end()) continue;
        out[f] = &it->second;
        ++found;
    }
    return out;
}

/**
 * A tagged tet mesh remeshed with its tag interfaces recorded. The tags are a check the map does
 * not see: every interface face separates two tag sets, and a point of the output mapped to the
 * input has to land on a face between the same two.
 */
void check_tags(
    const std::vector<std::string>& models,
    const nlohmann::json& options,
    const std::string& name)
{
    std::vector<std::string> inputs;
    for (const auto& m : models) {
        inputs.push_back((fs::path(WMTK_DATA_DIR) / "models" / m).string());
        REQUIRE(fs::exists(inputs.back()));
    }
    const fs::path dir = fs::current_path() / "simwild_history_test";
    fs::create_directories(dir);

    {
        const fs::path out = dir / name;
        const fs::path hist = dir / (name + ".wmtkhist");
        nlohmann::json json = {
            {"application", "simwild"},
            {"operation", "remeshing"},
            {"input", inputs},
            {"output", out.string()},
            {"history_output", hist.string()},
        };
        json.update(options);
        simwild::simwild(json);
        // The mesh the history starts from: the input itself for a .msh, its embedding otherwise.
        const auto in = interfaces(simwild::read_image_msh(out.string() + "_history_input.msh"));
        REQUIRE(!in.empty());
        const auto h = history::SurfaceHistory::load(hist.string());
        const surface_map::SurfaceMap map(h);
        const auto o = interfaces(simwild::read_image_msh(out.string() + ".msh"));
        REQUIRE(!o.empty());

        // The history starts at the input's interfaces and ends at the output's.
        size_t found_in = 0, found_out = 0;
        const auto in_labels = labels_of(
            map.input_vertices(),
            map.input_faces(),
            map.input_face_alive(),
            in,
            found_in);
        const auto out_labels = labels_of(
            map.output_vertices(),
            map.output_faces(),
            map.output_face_alive(),
            o,
            found_out);
        CHECK(found_in == in.size());
        CHECK(
            std::count(map.input_face_alive().begin(), map.input_face_alive().end(), 1) ==
            long(in.size()));
        CHECK(found_out == o.size());
        CHECK(
            std::count(map.output_face_alive().begin(), map.output_face_alive().end(), 1) ==
            long(o.size()));

        // Every output face's centre lands on the input between the same two tag sets.
        std::vector<surface_map::SurfacePoint> pts;
        std::vector<uint64_t> from;
        for (uint64_t f = 0; f < map.output_faces().size(); ++f) {
            if (!map.output_face_alive()[f] || !out_labels[f]) continue;
            pts.push_back({f, Eigen::Vector3d::Constant(1. / 3)});
            from.push_back(f);
        }
        const auto back = map.to_input(pts);
        size_t lost = 0, wrong = 0;
        for (size_t i = 0; i < back.size(); ++i) {
            if (back[i].face >= in_labels.size() || !in_labels[back[i].face]) {
                ++lost;
            } else if (*in_labels[back[i].face] != *out_labels[from[i]]) {
                ++wrong;
            }
        }
        const auto& w = map.build_warnings();
        INFO(
            "build warnings " << w.total() << " (not a disk " << w.not_a_disk << ", flat "
                              << w.flat_triangle << ", inverted " << w.inverted << ", merged "
                              << w.merged_faces << ", replaced " << w.replaced << "), "
                              << map.map_warnings().unlocated << " unlocated; " << lost << " lost, "
                              << wrong << " on the wrong interface, of " << back.size());
        CHECK(lost == 0);
        CHECK(wrong == 0);
        CHECK(w.replaced == 0);
    }
}

} // namespace

TEST_CASE("simwild-surface-history-keeps-tags", "[simwild][history]")
{
    for (const int threads : {0, 4}) {
        INFO("threads " << threads);
        check_tags(
            {"double_sphere.msh"},
            {{"max_iterations", 3}, {"num_threads", threads}},
            fmt::format("double_sphere_{}", threads));
    }
}

TEST_CASE("simwild-surface-history-keeps-tags-fine", "[simwild][history][.]")
{
    check_tags(
        {"double_sphere.msh"},
        {{"max_iterations", 10}, {"num_threads", 8}, {"length_rel", 0.02}},
        "double_sphere_fine");
}

TEST_CASE("simwild-surface-history-keeps-tags-free-topology", "[simwild][history][.]")
{
    // Without preserve_topology, update_attributes re-derives the surface from the tags between
    // passes; the history has to follow.
    check_tags(
        {"double_sphere.msh"},
        {{"max_iterations", 10}, {"num_threads", 8}, {"preserve_topology", false}},
        "double_sphere_free");
}

TEST_CASE("simwild-surface-history-keeps-tags-127891", "[simwild][history][.]")
{
    check_tags({"127891.msh"}, {{"max_iterations", 10}, {"num_threads", 8}}, "127891");
}

namespace {
/// Two unit spheres, the second moved along x: four tag sets, interfaces meeting along a circle.
const nlohmann::json two_spheres = {
    {"input_transform",
     {nlohmann::json::array(), {{1, 0, 0, 0.3}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}}}},
    {"eps_rel", 1e-2},
    {"stop_energy", 10},
};
} // namespace

TEST_CASE("simwild-surface-history-keeps-tags-embedded", "[simwild][history]")
{
    // A surface input: the history starts at its embedding, which the operations then remesh hard.
    for (const int threads : {0, 4}) {
        INFO("threads " << threads);
        nlohmann::json options = two_spheres;
        options.update({{"max_iterations", 3}, {"num_threads", threads}});
        check_tags({"sphere.obj", "sphere.obj"}, options, fmt::format("two_spheres_{}", threads));
    }
}
