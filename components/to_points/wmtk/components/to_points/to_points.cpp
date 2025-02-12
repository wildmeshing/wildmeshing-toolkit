#include "to_points.hpp"
#include <Eigen/Geometry>


#include <bitset>

#include <SimpleBVH/BVH.hpp>
#include <wmtk/PointMesh.hpp>
#include <wmtk/simplex/faces_single_dimension.hpp>
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>

namespace wmtk::components {

using namespace simplex;

std::shared_ptr<PointMesh> to_points(
    const Mesh& mesh,
    const attribute::MeshAttributeHandle& pts_attr,
    const ToPtsOptions& options,
    const std::string& output_pos_attr_name)
{
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

    wmtk::logger().info(
        "Input bbox max {}, min {}, diag {}, potential target edge length: {}",
        bbox.max(),
        bbox.min(),
        bbox.diagonal().norm(),
        bbox.diagonal().norm() * options.box_scale);

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
        const double bbox_diag = bbox.diagonal().norm();
        const auto r = Eigen::VectorXd::Ones(pts_acc.dimension()) * bbox_diag;
        const double grid_spacing = options.grid_spacing;


        bbox.min() -= options.box_scale * r;
        bbox.max() += options.box_scale * r;
        // Eigen::VectorXi res = (diag / grid_spacing).cast<int>();

        // // TODO: remove the hack
        // // hack grid spacing as relative
        const Eigen::VectorXi res = (bbox.diagonal() / (bbox_diag * grid_spacing)).cast<int>() +
                                    Eigen::VectorXi::Ones(pts_acc.dimension());

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
                        const Eigen::Vector3d ttmp = bbox.diagonal().array() * iii.array();
                        background_V.row(v_index(i, j, k)) =
                            bbox.min().array() + ttmp.array() / res.cast<double>().array();
                    }
                }
            }
        }

        wmtk::logger().info(
            "Grid bbox max {}, min {}, diag {}, potential target edge length: {}",
            bbox.max(),
            bbox.min(),
            bbox.diagonal().norm(),
            bbox.diagonal().norm() * options.box_scale);
        // else 2d and 1d

        if (options.min_dist >= 0) {
            int64_t count = 0;
            int64_t findex = 0;

            const std::vector<Tuple>& facest = mesh.get_all(mesh.top_simplex_type());
            const int64_t dim = int64_t(mesh.top_simplex_type()) + 1;

            Eigen::MatrixXd vertices(dim * facest.size(), pts_acc.dimension());
            Eigen::MatrixXi faces(facest.size(), dim);

            for (const auto& f : facest) {
                auto tmp = faces_single_dimension_tuples(
                    mesh,
                    simplex::Simplex(mesh, mesh.top_simplex_type(), f),
                    PrimitiveType::Vertex);

                assert(tmp.size() == dim);
                for (int64_t j = 0; j < tmp.size(); ++j) {
                    auto p = pts_acc.const_vector_attribute(tmp[j]);
                    faces(findex, j) = count;
                    vertices.row(dim * findex + j) = p;

                    ++count;
                }
                ++findex;
            }

            SimpleBVH::BVH bvh;
            bvh.init(vertices, faces, 1e-10);

            const double min_dist =
                options.min_dist > 0 ? (options.min_dist * options.min_dist * bbox_diag * bbox_diag)
                                     : (bbox_diag * bbox_diag * grid_spacing * grid_spacing / 4);
            std::vector<Eigen::VectorXd> good;
            SimpleBVH::VectorMax3d nearest_point;
            for (int64_t i = 0; i < background_V.rows(); ++i) {
                double sq_dist;
                bvh.nearest_facet(background_V.row(i), nearest_point, sq_dist);
                if (sq_dist >= min_dist) good.emplace_back(background_V.row(i));
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

    // remove duplicates
    int64_t old_size = pts.rows();
    auto remove_duplicated_vertices = [](Eigen::MatrixXd& P) -> Eigen::MatrixXd {
        std::vector<Eigen::VectorXd> vec;
        for (int64_t i = 0; i < P.rows(); ++i) vec.push_back(P.row(i));

        std::sort(vec.begin(), vec.end(), [](Eigen::VectorXd const& p1, Eigen::VectorXd const& p2) {
            return (p1(0) < p2(0)) || (p1(0) == p2(0) && p1(1) < p2(1)) ||
                   (p1(0) == p2(0) && p1(1) == p2(1) && p1(2) < p2(2));
        });

        auto it = std::unique(vec.begin(), vec.end());
        vec.resize(std::distance(vec.begin(), it));

        Eigen::MatrixXd new_P(vec.size(), P.cols());
        for (int64_t i = 0; i < vec.size(); ++i) {
            new_P.row(i) = vec[i];
        }

        return new_P;
    };

    if (options.remove_duplicates) {
        pts = remove_duplicated_vertices(pts);
        wmtk::logger().info("removed {} duplicated vertices", pts.rows() - old_size);
    }

    wmtk::logger().info("generated {} vertices", pts.rows());
    std::shared_ptr<PointMesh> pts_mesh = std::make_shared<PointMesh>(pts.rows());

    mesh_utils::set_matrix_attribute(pts, output_pos_attr_name, PrimitiveType::Vertex, *pts_mesh);

    return pts_mesh;
}

} // namespace wmtk::components
