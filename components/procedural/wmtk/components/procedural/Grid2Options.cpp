#include "Grid2Options.hpp"
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "grid_utils.hpp"


namespace wmtk::components::procedural {
const std::array<std::string, 2> Grid2Options::tiling_names = {{"bcc", "diagonal"}};

namespace {
std::shared_ptr<TriMesh> make_diagonal_mesh(const Grid2Options& opt)
{
    const auto d = opt.dimensions;
    auto vertex_dimensions = opt.dimensions;
    for (size_t j = 0; j < d.size(); ++j) {
        int64_t& v = vertex_dimensions[j];
        if (!opt.cycles[j]) {
            v++;
        }
    }
    using CoordType = std::array<int64_t, 2>;

    Eigen::Matrix<int64_t, Eigen::Dynamic, 3> FV(2 * d[0] * d[1], 3);

    CoordType i;
    // auto& [j, k] = i;
    int64_t& j = i[0];
    int64_t& k = i[1];
    for (j = 0; j < d[0]; ++j) {
        for (k = 0; k < d[1]; ++k) {
            auto f = [&](int64_t a, int64_t b) {
                CoordType coord{{j + a, k + b}};
                for (size_t e = 0; e < 2; ++e) {
                    auto& v = coord[e];
                    v = v % vertex_dimensions[e];
                }
                return procedural::grid_index(vertex_dimensions, coord);
            };
            int64_t c[2][2] = {
                {
                    f(0, 0),
                    f(1, 0),
                },
                {
                    f(0, 1),
                    f(1, 1),
                }};
            int64_t f0_index = 2 * procedural::grid_index(d, i);
            int64_t f1_index = f0_index + 1;

            FV.row(f0_index) << c[0][0], c[0][1], c[1][0];
            FV.row(f1_index) << c[1][1], c[1][0], c[0][1];
        }
    }

    auto m = std::make_shared<TriMesh>();
    m->initialize(FV);
    if (opt.coordinates.has_value()) {
        int vertex_size = vertex_dimensions[0] * vertex_dimensions[1];

        const auto& spacing = opt.coordinates->spacing;
        Eigen::Matrix<double, Eigen::Dynamic, 2> P(vertex_size, 2);
        for (j = 0; j < vertex_dimensions[0]; ++j) {
            double x = spacing[0] * j;
            for (k = 0; k < vertex_dimensions[1]; ++k) {
                double y = spacing[1] * k;
                int64_t index = procedural::grid_index(vertex_dimensions, i);

                P.row(index) << x, y;
            }
        }
        const auto& name = opt.coordinates->name;
        wmtk::mesh_utils::set_matrix_attribute(P, name, PrimitiveType::Vertex, *m);
    }
    return m;
}
} // namespace

std::shared_ptr<TriMesh> make_mesh(const Grid2Options& opt)
{
    switch (opt.tiling_type) {
    case Grid2Options::TilingType::Diagonal: return make_diagonal_mesh(opt);
    case Grid2Options::TilingType::BCC:
        throw std::runtime_error("bcc lattice not implemented yet");
        [[fallthrough]];
    default: break;
    }
    throw std::runtime_error("failed to select a tiling type");
    return nullptr;
}

    void to_json(nlohmann::json& nlohmann_json_j, const Grid2Options& nlohmann_json_t)
    {
        nlohmann_json_j["tiling"] = Grid2Options::tiling_names[static_cast<size_t>(nlohmann_json_t.tiling_type)];
        if (nlohmann_json_t.coordinates.has_value()) {
            nlohmann_json_j["coordinates"] = *nlohmann_json_t.coordinates;
        }
        nlohmann_json_j["dimensions"] = nlohmann_json_t.dimensions;
        {
            const auto& b = nlohmann_json_t.cycles;
            std::array<bool, 2> bs{{b[0], b[1]}};
            nlohmann_json_j["cycles"] = bs;
        }
    }
    void from_json(const nlohmann::json& nlohmann_json_j, Grid2Options& nlohmann_json_t)
    {
        nlohmann_json_t.dimensions = nlohmann_json_j["dimensions"].get<std::array<int64_t, 2>>();

        {
            const std::string tiling = nlohmann_json_j["tiling"];
            bool found = false;
            for (size_t j = 0; j < Grid2Options::tiling_names.size(); ++j) {
                if (tiling == Grid2Options::tiling_names[j]) {
                    found = true;
                    nlohmann_json_t.tiling_type = static_cast<Grid2Options::TilingType>(j);
                }
            }
            if (!found) {
                throw std::runtime_error(fmt::format(
                    "Tiling type was not found, got [{}], expected one of {{[{}]}}",
                    tiling,
                    fmt::join(Grid2Options::tiling_names, "],[")));
            }
        }
        {
            auto& b = nlohmann_json_t.cycles;
            const auto& c = nlohmann_json_j["cycles"];
            for (int j = 0; j < 2; ++j) {
                b[j] = c[j];
            }
        }
        if (const auto& coords = nlohmann_json_j["coordinates"]; !coords.is_null()) {
            if (nlohmann_json_j["coordinates"]["spacing"][0] > 0)
                nlohmann_json_t.coordinates = coords.get<Grid2Options::Coordinates>();
        }
    }
} // namespace wmtk::components::procedural
