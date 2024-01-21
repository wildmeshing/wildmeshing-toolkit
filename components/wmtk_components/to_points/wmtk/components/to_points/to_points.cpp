#include "to_points.hpp"


#include "ToPtsOptions.hpp"

#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk::components {

void to_points(const base::Paths& paths, const nlohmann::json& j, io::Cache& cache)
{
    ToPtsOptions options = j.get<ToPtsOptions>();

    const std::shared_ptr<Mesh> mesh_in = cache.read_mesh(options.input);
    const Mesh& mesh = *mesh_in;

    const auto pts_attr =
        mesh.get_attribute_handle<double>(options.position, PrimitiveType::Vertex);
    const auto pts_acc = mesh.create_const_accessor<double>(pts_attr);
    const auto vs = mesh.get_all(PrimitiveType::Vertex);

    Eigen::AlignedBox<double, Eigen::Dynamic> bbox(pts_acc.dimension());

    int64_t box_size = std::pow(2, pts_acc.dimension());


    Eigen::MatrixXd pts(vs.size() + (options.add_box ? box_size : 0), pts_acc.dimension());
    int64_t index = 0;
    for (const auto& v : vs) {
        pts.row(index) << pts_acc.const_vector_attribute(v).transpose();
        bbox.extend(pts.row(index).transpose());
        ++index;
    }

    if (options.add_box) {
        auto center = bbox.center();
        auto r = bbox.diagonal() / 2.;

        bbox.min() = center - options.box_scale * r;
        bbox.max() = center + options.box_scale * r;

        for (int64_t d = 0; d < box_size; ++d) {
            std::bitset<32> bits = d;
            for (int64_t i = 0; i < pts.cols(); ++i) {
                pts(index, i) = bits[i] == 0 ? bbox.min()[i] : bbox.max()[i];
            }
            ++index;
        }
    }

    PointMesh pts_mesh(pts.rows());

    mesh_utils::set_matrix_attribute(pts, options.position, PrimitiveType::Vertex, pts_mesh);

    cache.write_mesh(pts_mesh, options.name);
}

} // namespace wmtk::components