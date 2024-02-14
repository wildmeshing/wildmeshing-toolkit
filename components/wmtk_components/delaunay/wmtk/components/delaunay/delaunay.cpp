#include "delaunay.hpp"

#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include "internal/DelaunayOptions.hpp"
#include "internal/delaunay_2d.hpp"
#include "internal/delaunay_3d.hpp"

namespace wmtk::components {

template <int D>
RowVectors<double, D> points_to_rowvectors(const std::string& position, PointMesh& point_cloud)
{
    auto pts_attr = point_cloud.get_attribute_handle<double>(position, PrimitiveType::Vertex);
    auto pts_acc = point_cloud.create_accessor<double>(pts_attr);

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
void delaunay_exec(const internal::DelaunayOptions& options, io::Cache& cache)
{
    // 2d --> TriMesh
    // 3d --> TetMesh
    static_assert(
        (D == 2 && std::is_same<MeshT, TriMesh>()) || (D == 3 && std::is_same<MeshT, TetMesh>()));

    // input
    std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);
    if (mesh_in->top_simplex_type() != PrimitiveType::Vertex) {
        log_and_throw_error(
            "Delaunay works only for point meshes: {}",
            mesh_in->top_simplex_type());
    }

    PointMesh& point_cloud = static_cast<PointMesh&>(*mesh_in);

    // make sure dimensions fit
    {
        auto pts_attr =
            point_cloud.get_attribute_handle<double>(options.position, PrimitiveType::Vertex);
        auto pts_acc = point_cloud.create_accessor<double>(pts_attr);
        assert(pts_acc.dimension() == options.cell_dimension);
    }

    if constexpr (D == 2) {
        throw std::runtime_error("not tested for 2d");
    }

    std::shared_ptr<MeshT> meshptr = std::make_shared<MeshT>();
    MeshT& mesh = *meshptr;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    const auto pts_vec = points_to_rowvectors<D>(options.position, point_cloud);
    if constexpr (D == 2) {
        std::tie(vertices, faces) = internal::delaunay_2d(pts_vec);
    } else if constexpr (D == 3) {
        std::tie(vertices, faces) = internal::delaunay_3d(pts_vec);
    } else {
        throw std::runtime_error("unsupported cell dimension in delaunay component");
    }

    mesh.initialize(faces.cast<int64_t>());
    mesh_utils::set_matrix_attribute(vertices, options.position, PrimitiveType::Vertex, mesh);

    cache.write_mesh(mesh, options.output);
}

void delaunay(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    using namespace internal;

    DelaunayOptions options = j.get<DelaunayOptions>();

    // delaunay
    switch (options.cell_dimension) {
    case 2: {
        delaunay_exec<2, TriMesh>(options, cache);
        break;
    }
    case 3: {
        delaunay_exec<3, TetMesh>(options, cache);
        break;
    }
    default: {
        throw std::runtime_error("unsupported cell dimension in delaunay component");
    }
    }
}

} // namespace wmtk::components
