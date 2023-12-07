#include "delaunay.hpp"

#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/io/HDF5Writer.hpp>
#include <wmtk/io/MeshReader.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include "internal/DelaunayOptions.hpp"
#include "internal/delaunay_2d.hpp"
#include "internal/delaunay_3d.hpp"

namespace wmtk {
namespace components {

template <int D>
RowVectors<double, D> points_to_rowvectors(PointMesh& point_cloud)
{
    auto pts_attr = point_cloud.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
    auto pts_acc = point_cloud.create_accessor(pts_attr);

    const auto vertices = point_cloud.get_all(PrimitiveType::Vertex);

    RowVectors<double, D> vec(vertices.size(), D);
    size_t i = 0;
    for (const Tuple& t : vertices) {
        const auto p = pts_acc.vector_attribute(t);
        vec.row(i) = p.transpose();
        ++i;
    }

    return vec;
}

template <int D, typename MeshT>
void delaunay_exec(
    const internal::DelaunayOptions& options,
    std::map<std::string, std::filesystem::path>& files)
{
    // 2d --> TriMesh
    // 3d --> TetMesh
    static_assert(
        (D == 2 && std::is_same<MeshT, TriMesh>()) || (D == 3 && std::is_same<MeshT, TetMesh>()));

    // input
    const std::filesystem::path& file = files[options.input];
    std::shared_ptr<Mesh> mesh_in = read_mesh(file);
    if (mesh_in->top_simplex_type() != PrimitiveType::Vertex) {
        log_and_throw_error(
            "Delaunay works only for point meshes: {}",
            mesh_in->top_simplex_type());
    }

    PointMesh& point_cloud = static_cast<PointMesh&>(*mesh_in);

    // make sure dimensions fit
    {
        auto pts_attr = point_cloud.get_attribute_handle<double>("vertices", PrimitiveType::Vertex);
        auto pts_acc = point_cloud.create_accessor(pts_attr);
        assert(pts_acc.dimension() == options.cell_dimension);
    }

    if constexpr (D == 2) {
        throw std::runtime_error("not tested for 2d");
    }

    MeshT mesh;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    const auto pts_vec = points_to_rowvectors<D>(point_cloud);
    if constexpr (D == 2) {
        std::tie(vertices, faces) = internal::delaunay_2d(pts_vec);
    } else if constexpr (D == 3) {
        std::tie(vertices, faces) = internal::delaunay_3d(pts_vec);
    } else {
        throw std::runtime_error("unsupported cell dimension in delaunay component");
    }

    mesh.initialize(faces.cast<long>());
    mesh_utils::set_matrix_attribute(vertices, "vertices", PrimitiveType::Vertex, mesh);

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
        delaunay_exec<2, TriMesh>(options, files);
        break;
    }
    case 3: {
        delaunay_exec<3, TetMesh>(options, files);
        break;
    }
    default: {
        throw std::runtime_error("unsupported cell dimension in delaunay component");
    }
    }
}
} // namespace components
} // namespace wmtk
