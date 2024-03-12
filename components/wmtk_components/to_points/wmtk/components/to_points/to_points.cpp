#include "to_points.hpp"


#include "ToPtsOptions.hpp"

#include <bitset>

#include <SimpleBVH/BVH.hpp>
#include <wmtk/Mesh.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk::components {

using namespace simplex;

void to_points(const base::Paths& paths, const nlohmann::json& json, io::Cache& cache)
{
    ToPtsOptions options = json.get<ToPtsOptions>();

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


    if (options.add_box && !options.add_grid) {
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

    if (options.add_grid) {
        auto center = bbox.center();
        auto r = bbox.diagonal() / 2.;
        const double grid_spacing = options.grid_spacing;

        bbox.min() = center - options.box_scale * r;
        bbox.max() = center + options.box_scale * r;
        Eigen::VectorXd diag = bbox.max() - bbox.min();
        Eigen::VectorXi res = (diag / grid_spacing).cast<int>();
        Eigen::MatrixXd background_V;


        if (pts.cols() == 3) {
            auto v_index = [&res](int i, int j, int k) {
                return k * (res[0] + 1) * (res[1] + 1) + j * (res[0] + 1) + i;
            };

            background_V.resize((res[0] + 1) * (res[1] + 1) * (res[2] + 1), 3);

            for (int k = 0; k <= res[2]; ++k) {
                for (int j = 0; j <= res[1]; ++j) {
                    for (int i = 0; i <= res[0]; ++i) {
                        const Eigen::Vector3d iii(i, j, k);
                        const Eigen::Vector3d ttmp = diag.array() * iii.array();
                        background_V.row(v_index(i, j, k)) =
                            bbox.min().array() + ttmp.array() / res.cast<double>().array();
                    }
                }
            }
        }
        // else 2d and 1d

        if (options.min_dist > 0) {
            int64_t count = 0;
            int64_t index = 0;

            const std::vector<Tuple>& facest = mesh.get_all(mesh.top_simplex_type());
            const int64_t dim = int64_t(mesh.top_simplex_type()) + 1;

            Eigen::MatrixXd vertices(dim * facest.size(), pts_acc.dimension());
            Eigen::MatrixXi faces(facest.size(), dim);

            for (const auto& f : facest) {
                auto tmp = faces_single_dimension_tuples(
                    mesh,
                    simplex::Simplex(mesh.top_simplex_type(), f),
                    PrimitiveType::Vertex);

                assert(tmp.size() == dim);
                for (int64_t j = 0; j < tmp.size(); ++j) {
                    auto p = pts_acc.const_vector_attribute(tmp[j]);
                    faces(index, j) = count;
                    vertices.row(dim * index + j) = p;

                    ++count;
                }
                ++index;
            }

            SimpleBVH::BVH bvh;
            bvh.init(vertices, faces, 1e-10);

            std::vector<Eigen::VectorXd> good;
            SimpleBVH::VectorMax3d nearest_point;
            for (int64_t i = 0; i < background_V.rows(); ++i) {
                double sq_dist;
                bvh.nearest_facet(background_V.row(i), nearest_point, sq_dist);
                if (sq_dist >= options.min_dist) good.emplace_back(background_V.row(i));
            }
            int64_t current_size = pts.rows();
            pts.conservativeResize(current_size + good.size(), pts.cols());
            for (const auto& v : good) {
                pts.row(current_size) = v.transpose();
                ++current_size;
            }
        } else {
            // TODO
        }
    }
    std::shared_ptr<PointMesh> pts_mesh = std::make_shared<PointMesh>(pts.rows());

    mesh_utils::set_matrix_attribute(pts, options.position, PrimitiveType::Vertex, *pts_mesh);

    cache.write_mesh(*pts_mesh, options.name);
}

} // namespace wmtk::components
