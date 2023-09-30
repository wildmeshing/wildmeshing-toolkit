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

template <int D>
RowVectors<double, D> points_to_rowvectors(PointMesh& point_cloud)
{
    auto pts_attr = point_cloud.get_attribute_handle<double>("position", PrimitiveType::Vertex);
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


void delaunay(const nlohmann::json& j, std::map<std::string, std::filesystem::path>& files)
{
    using namespace internal;

    DelaunayOptions options = j.get<DelaunayOptions>();

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

    // delaunay
    switch (options.cell_dimension) {
    case 2: {
        throw "not tested";
        TriMesh mesh;
        Eigen::MatrixXd vertices;
        Eigen::MatrixXi faces;
        auto pts_vec = points_to_rowvectors<2>(point_cloud);
        internal::delaunay_2d(pts_vec, vertices, faces);

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

        break;
    }
    case 3: {
        TetMesh mesh;
        Eigen::MatrixXd vertices;
        Eigen::MatrixXi tetrahedra;
        auto pts_vec = points_to_rowvectors<3>(point_cloud);
        internal::delaunay_3d(pts_vec, vertices, tetrahedra);

        mesh.initialize(tetrahedra.cast<long>());
        mesh_utils::set_matrix_attribute(vertices, "position", PrimitiveType::Vertex, mesh);

        // output
        {
            const std::filesystem::path cache_dir = "cache";
            const std::filesystem::path cached_mesh_file = cache_dir / (options.output + ".hdf5");

            HDF5Writer writer(cached_mesh_file);
            mesh.serialize(writer);

            files[options.output] = cached_mesh_file;
        }

        break;
    }
    default: {
        throw "unsupported cell dimension in delaunay component";
        break;
    }
    }
}
} // namespace components
} // namespace wmtk
