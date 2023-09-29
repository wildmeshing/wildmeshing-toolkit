#include "delaunay.hpp"

#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include "internal/DelaunayOptions.hpp"
#include "internal/delaunay_2d.hpp"
#include "internal/delaunay_3d.hpp"

namespace wmtk {
namespace components {

template <typename T>
std::vector<T> points_to_vector(PointMesh& point_cloud)
{
    auto pts_attr = point_cloud.get_attribute_handle<double>("position", PrimitiveType::Vertex);
    auto pts_acc = point_cloud.create_accessor(pts_attr);

    const auto vertices = point_cloud.get_all(PrimitiveType::Vertex);

    assert(pts_acc.dimension() == T().rows());

    std::vector<T> vec;
    vec.reserve(vertices.size());
    for (const Tuple& t : vertices) {
        vec.emplace_back(pts_acc.vector_attribute(t));
    }

    return vec;
}

template <typename VectorT, typename MeshT>
void delaunay_exec(
    const internal::DelaunayOptions& options,
    std::map<std::string, std::filesystem::path>& files)
{
    // 2d --> TriMesh
    // 3d --> TetMesh
    static_assert(
        (std::is_same<VectorT, Eigen::Vector2d>() && std::is_same<MeshT, TriMesh>()) ||
        (std::is_same<VectorT, Eigen::Vector3d>() && std::is_same<MeshT, TetMesh>()));

    // input
    PointMesh point_cloud;
    {
        const std::filesystem::path& file = files[options.input];
        MeshReader reader(file);
        reader.read(point_cloud);
    }

    // make sure dimensions fit
    {
        auto pts_attr = point_cloud.get_attribute_handle<double>("position", PrimitiveType::Vertex);
        auto pts_acc = point_cloud.create_accessor(pts_attr);
        assert(pts_acc.dimension() == options.cell_dimension);
    }

    if constexpr (std::is_same<VectorT, Eigen::Vector2d>()) {
        throw "not tested for 2d";
    }

    MeshT mesh;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    const auto pts_vec = points_to_vector<VectorT>(point_cloud);
    if constexpr (std::is_same<VectorT, Eigen::Vector2d>()) {
        std::tie(vertices, faces) = internal::delaunay_2d(pts_vec);
    } else if constexpr (std::is_same<VectorT, Eigen::Vector3d>()) {
        std::tie(vertices, faces) = internal::delaunay_3d(pts_vec);
    } else {
        throw "unsupported cell dimension in delaunay component";
    }

    mesh.initialize(faces.cast<long>());
    mesh_utils::set_matrix_attribute(vertices, "position", PrimitiveType::Vertex, mesh);

    // output
    {
        const std::filesystem::path cache_dir = "cache";
        const std::filesystem::path cached_mesh_file = cache_dir / (options.output + ".hdf5");

        HDF5Writer writer(cached_mesh_file);
        mesh.serialize(writer);

        files[options.output] = cached_mesh_file;
    }
}

void delaunay(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    DelaunayOptions options = j.get<DelaunayOptions>();

    // delaunay
    switch (options.cell_dimension) {
    case 2: {
        delaunay_exec<Eigen::Vector2d, TriMesh>(options, files);
        break;
    }
    case 3: {
        delaunay_exec<Eigen::Vector3d, TetMesh>(options, files);
        break;
    }
    default: {
        throw "unsupported cell dimension in delaunay component";
    }
    }
}
} // namespace components
} // namespace wmtk
