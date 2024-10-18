#include "DiskOptions.hpp"
#include <wmtk/Mesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include "grid_utils.hpp"


namespace wmtk::components::procedural {

std::shared_ptr<Mesh> make_mesh(const DiskOptions& opt)
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

    tris(size - 1, 2) = 1;
    m.initialize(tris);
    if (opt.coordinates.has_value()) {
        const auto& csettings = *opt.coordinates;
        int vertex_size = size + 1;
        double radius = csettings.radius;
        const auto& name = csettings.name;
        double dtheta = 2.0 * M_PI / size;
        double theta0 = M_PI * csettings.degree_offset / 180.0;


        Eigen::Matrix<double, Eigen::Dynamic, 2> P(vertex_size, 2);
        P.row(0).setZero();
        for (int j = 0; j < size; ++j) {
            double theta = theta0 + dtheta * j;
            double c = std::cos(theta);
            double s = std::sin(theta);
            P.row(j + 1) << c, s;
        }
        Eigen::Vector2d::ConstMapType o(csettings.center.data());
        P.rowwise() += o.transpose();
        wmtk::mesh_utils::set_matrix_attribute(P, name, PrimitiveType::Vertex, m);
    }
    return mptr;
}

    void to_json(nlohmann::json& nlohmann_json_j, const DiskOptions& nlohmann_json_t)
    {
        nlohmann_json_j["size"] = nlohmann_json_t.size;
        if (nlohmann_json_t.coordinates.has_value()) {
            nlohmann_json_j["coordinates"] = *nlohmann_json_t.coordinates;
        }
    }
    void from_json(const nlohmann::json& nlohmann_json_j, DiskOptions& nlohmann_json_t)
    {
        nlohmann_json_t.size = nlohmann_json_j["size"];
        if (const auto& coords = nlohmann_json_j["coordinates"]; !coords.is_null()) {
            nlohmann_json_t.coordinates = coords.get<DiskOptions::Coordinates>();
        }
    }
} // namespace wmtk::components::procedural
