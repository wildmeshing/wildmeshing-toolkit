#include "TriangleFanOptions.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "grid_utils.hpp"


namespace wmtk::components::procedural {

std::shared_ptr<TriMesh> make_mesh(const TriangleFanOptions& opt)
{
    const size_t size = opt.size;

    auto mptr = std::make_shared<TriMesh>();
    TriMesh& m = *mptr;
    RowVectors3l tris;
    tris.resize(size, 3);
    tris.rowwise() = Vector3l(0, 1, 2).transpose();
    auto mut = tris.rightCols<2>();
    for (int j = 0; j < size; ++j) {
        mut.row(j).array() += j;
    }

    // tris(number - 1, 2) = 1;
    m.initialize(tris);
    if (opt.coordinates.has_value()) {
        int vertex_size = size + 2;
        const auto& csettings = *opt.coordinates;
        const auto& name = csettings.name;
        double radius = csettings.radius;
        Eigen::Vector2d::ConstMapType degree_range(csettings.degrees.data());
        double diff = degree_range.y() - degree_range.x();
        double dtheta = diff * M_PI / 180.0 / size;
        double theta0 = M_PI * degree_range.x() / 18.0;


        Eigen::Matrix<double, Eigen::Dynamic, 2> P(vertex_size, 2);
        P.row(0).setZero();
        for (int j = 0; j <= size; ++j) {
            double theta = dtheta * j + theta0;
            double c = radius * std::cos(theta);
            double s = radius * std::sin(theta);
            P.row(j + 1) << c, s;
        }
        Eigen::Vector2d::ConstMapType o(csettings.center.data());
        P.rowwise() += o.transpose();
        wmtk::mesh_utils::set_matrix_attribute(P, name, PrimitiveType::Vertex, m);
    }
    return mptr;
}

        void to_json(nlohmann::json& nlohmann_json_j, const TriangleFanOptions::Coordinates& nlohmann_json_t)
        {
            NLOHMANN_JSON_EXPAND(
                NLOHMANN_JSON_PASTE(NLOHMANN_JSON_TO, name, radius, center, degrees));
        }
        void from_json(const nlohmann::json& nlohmann_json_j, TriangleFanOptions::Coordinates& nlohmann_json_t)
        {
            NLOHMANN_JSON_EXPAND(NLOHMANN_JSON_PASTE(NLOHMANN_JSON_FROM, name, radius, center));
            assert(nlohmann_json_j.contains("degrees"));
            if (const auto& deg = nlohmann_json_j["degrees"]; deg.is_number()) {
                nlohmann_json_t.degrees = std::array<double, 2>{{0.0, double(deg)}};
            } else {
                nlohmann_json_t.degrees = deg.get<std::array<double, 2>>();
            }
        }

    void to_json(nlohmann::json& nlohmann_json_j, const TriangleFanOptions& nlohmann_json_t)
    {
        nlohmann_json_j["size"] = nlohmann_json_t.size;
        if (nlohmann_json_t.coordinates.has_value()) {
            nlohmann_json_j["coordinates"] = *nlohmann_json_t.coordinates;
        }
    }
    void from_json(
        const nlohmann::json& nlohmann_json_j,
        TriangleFanOptions& nlohmann_json_t)
    {
        nlohmann_json_t.size = nlohmann_json_j["size"];
        if (const auto& coords = nlohmann_json_j["coordinates"]; !coords.is_null()) {
            nlohmann_json_t.coordinates = coords.get<TriangleFanOptions::Coordinates>();
        }
    }
} // namespace wmtk::components::procedural
