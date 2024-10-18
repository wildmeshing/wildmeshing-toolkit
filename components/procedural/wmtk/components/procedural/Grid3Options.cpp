
#include "Grid3Options.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "grid_utils.hpp"


namespace wmtk::components::procedural {
const std::array<std::string, 2> Grid3Options::Grid3Options::tiling_names = {{"bcc", "freudenthal"}};

namespace {
std::shared_ptr<TetMesh> make_freudenthal_mesh(const Grid3Options& opt)
{
    const auto& d = opt.dimensions;
    auto vertex_dimensions = opt.dimensions;
    for (size_t j = 0; j < d.size(); ++j) {
        int64_t& v = vertex_dimensions[j];
        if (!opt.cycles[j]) {
            v++;
        }
    }
    using CoordType = std::array<int64_t, 3>;

    Eigen::Matrix<int64_t, Eigen::Dynamic, 4> FV(6 * d[0] * d[1] * d[2], 4);

    CoordType i;
    // auto& [j, k, l] = i;
    int64_t& j = i[0];
    int64_t& k = i[1];
    int64_t& l = i[2];
    for (j = 0; j < d[0]; ++j) {
        for (k = 0; k < d[1]; ++k) {
            for (l = 0; l < d[2]; ++l) {
                auto f = [&](int64_t a, int64_t b, int64_t c) {
                    CoordType coord{{j + a, k + b, l + c}};
                    for (size_t e = 0; e < 3; ++e) {
                        auto& v = coord[e];
                        v = v % vertex_dimensions[e];
                    }


                    return procedural::grid_index(vertex_dimensions, coord);
                };
                int64_t c[8] = {
                    f(0, 0, 0),
                    f(1, 0, 0),
                    f(1, 1, 0),
                    f(0, 1, 0),
                    f(0, 0, 1),
                    f(1, 0, 1),
                    f(1, 1, 1),
                    f(0, 1, 1),
                };
                int64_t f0_index = 6 * procedural::grid_index(d, i);


                FV.row(f0_index + 0) << c[0], c[1], c[3], c[4];
                FV.row(f0_index + 1) << c[5], c[2], c[6], c[7];
                FV.row(f0_index + 2) << c[4], c[1], c[5], c[3];
                FV.row(f0_index + 3) << c[4], c[3], c[7], c[5];
                FV.row(f0_index + 4) << c[3], c[1], c[5], c[2];
                FV.row(f0_index + 5) << c[2], c[3], c[7], c[5];
            }
        }
    }

    auto m = std::make_shared<TetMesh>();
    m->initialize(FV);
    if (opt.coordinates.has_value()) {
        int vertex_size = vertex_dimensions[0] * vertex_dimensions[1] * vertex_dimensions[2];

        const auto& spacing = opt.coordinates->spacing;
        Eigen::Matrix<double, Eigen::Dynamic, 3> P(vertex_size, 3);
        for (j = 0; j < vertex_dimensions[0]; ++j) {
            double x = spacing[0] * j;
            for (k = 0; k < vertex_dimensions[1]; ++k) {
                double y = spacing[1] * k;
                for (l = 0; l < vertex_dimensions[2]; ++l) {
                    double z = spacing[2] * l;
                    int64_t index = procedural::grid_index(vertex_dimensions, i);

                    P.row(index) << x, y, z;
                }
            }
        }
        const auto& name = opt.coordinates->name;
        wmtk::mesh_utils::set_matrix_attribute(P, name, PrimitiveType::Vertex, *m);
    }
    return m;
}
} // namespace

std::shared_ptr<TetMesh> make_mesh(const Grid3Options& opt)
{
    switch (opt.tiling_type) {
    case Grid3Options::TilingType::Freudenthal: return make_freudenthal_mesh(opt);
    case Grid3Options::TilingType::BCC:
        throw std::runtime_error("bcc lattice not implemented yet");
        [[fallthrough]];
    default: break;
    }
    throw std::runtime_error("failed to select a tiling type");
    return nullptr;
}

    void to_json(nlohmann::json& nlohmann_json_j, const Grid3Options& nlohmann_json_t)
    {
        nlohmann_json_j["tiling"] = Grid3Options::tiling_names[static_cast<size_t>(nlohmann_json_t.tiling_type)];
        if (nlohmann_json_t.coordinates.has_value()) {
            nlohmann_json_j["coordinates"] = *nlohmann_json_t.coordinates;
        }
        nlohmann_json_j["dimensions"] = nlohmann_json_t.dimensions;
        {
            const auto& b = nlohmann_json_t.cycles;
            std::array<bool, 3> bs{{b[0], b[1], b[2]}};
            nlohmann_json_j["cycles"] = bs;
        }
    }
    void from_json(const nlohmann::json& nlohmann_json_j, Grid3Options& nlohmann_json_t)
    {
        nlohmann_json_t.dimensions = nlohmann_json_j["dimensions"].get<std::array<int64_t, 3>>();
        {
            const std::string tiling = nlohmann_json_j["tiling"];
            bool found = false;
            for (size_t j = 0; j < Grid3Options::tiling_names.size(); ++j) {
                if (tiling == Grid3Options::tiling_names[j]) {
                    found = true;
                    nlohmann_json_t.tiling_type = static_cast<Grid3Options::TilingType>(j);
                }
            }
            if (!found) {
                throw std::runtime_error(fmt::format(
                    "Tiling type was not found, got [{}], expected one of {{[{}]}}",
                    tiling,
                    fmt::join(Grid3Options::tiling_names, "],[")));
            }
        }
        {
            auto& b = nlohmann_json_t.cycles;
            const auto& c = nlohmann_json_j["cycles"];
            for (int j = 0; j < 3; ++j) {
                b[j] = c[j];
            }
        }
        if (const auto& coords = nlohmann_json_j["coordinates"]; !coords.is_null()) {
            if (nlohmann_json_j["coordinates"]["spacing"][0] > 0)
                nlohmann_json_t.coordinates = coords.get<Grid3Options::Coordinates>();
        }
    }
} // namespace wmtk::components::procedural
