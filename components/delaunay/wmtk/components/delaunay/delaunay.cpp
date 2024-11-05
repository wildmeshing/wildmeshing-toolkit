#include "delaunay.hpp"

#include <wmtk/PointMesh.hpp>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>

#include "internal/delaunay_2d.hpp"
#include "internal/delaunay_3d.hpp"

namespace wmtk::components {

template <int D>
RowVectors<double, D> points_to_rowvectors(
    const PointMesh& point_cloud,
    const attribute::MeshAttributeHandle& pts_attr)
{
    auto pts_acc = point_cloud.create_const_accessor<double>(pts_attr);

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
std::shared_ptr<MeshT> delaunay_exec(
    const PointMesh& point_cloud,
    const attribute::MeshAttributeHandle& pts_attr,
    const std::string& output_pos_attr_name)
{
    // 2d --> TriMesh
    // 3d --> TetMesh
    static_assert(
        (D == 2 && std::is_same<MeshT, TriMesh>()) || (D == 3 && std::is_same<MeshT, TetMesh>()));

    if constexpr (D == 2) {
        throw std::runtime_error("not tested for 2d");
    }

    std::shared_ptr<MeshT> meshptr = std::make_shared<MeshT>();
    MeshT& mesh = *meshptr;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    const auto pts_vec = points_to_rowvectors<D>(point_cloud, pts_attr);
    if constexpr (D == 2) {
        std::tie(vertices, faces) = internal::delaunay_2d(pts_vec);
    } else if constexpr (D == 3) {
        std::tie(vertices, faces) = internal::delaunay_3d(pts_vec);
    } else {
        throw std::runtime_error("unsupported cell dimension in delaunay component");
    }

    mesh.initialize(faces.cast<int64_t>());
    mesh_utils::set_matrix_attribute(vertices, output_pos_attr_name, PrimitiveType::Vertex, mesh);

    return meshptr;
}

std::shared_ptr<Mesh> delaunay(
    const PointMesh& point_cloud,
    const attribute::MeshAttributeHandle& pts_attr,
    const std::string& output_pos_attr_name)
{
    using namespace internal;

    auto pts_acc = point_cloud.create_const_accessor<double>(pts_attr);
    const auto cell_dimension = pts_acc.dimension();


    // delaunay
    switch (cell_dimension) {
    case 2: {
        return delaunay_exec<2, TriMesh>(point_cloud, pts_attr, output_pos_attr_name);
        break;
    }
    case 3: {
        return delaunay_exec<3, TetMesh>(point_cloud, pts_attr, output_pos_attr_name);
        break;
    }
    default: {
        throw std::runtime_error("unsupported cell dimension in delaunay component");
    }
    }
}

} // namespace wmtk::components
