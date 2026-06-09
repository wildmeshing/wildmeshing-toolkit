#include "init_from_delaunay.hpp"

#include <bitset>
#include <wmtk/envelope/Envelope.hpp>
#include <wmtk/io/read_edge_mesh.hpp>
#include <wmtk/utils/Delaunay.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::triwild {

void init_from_delaunay_box_mesh(
    const MatrixXd& V,
    const MatrixXi& E,
    MatrixXd& V_out,
    MatrixXi& F_out,
    MatrixXi& E_out)
{
    assert(V.cols() == 2);

    // points for delaunay
    std::vector<wmtk::delaunay::Point2D> points(V.rows());
    // add points from surface
    for (int i = 0; i < V.rows(); i++) {
        for (int j = 0; j < 2; j++) {
            points[i][j] = V(i, j);
        }
    }

    // bbox
    Vector2d box_min = V.colwise().minCoeff();
    Vector2d box_max = V.colwise().maxCoeff();

    // increase bbox by 30% of diagonal length
    const double diagonal_length = (box_max - box_min).norm();
    const double delta = diagonal_length / 15.0;
    box_min -= Vector2d(delta, delta);
    box_max += Vector2d(delta, delta);

    // add corners of domain
    for (int i = 0; i < 4; i++) {
        Vector2d p;
        std::bitset<sizeof(int) * 4> a(i);
        for (int j = 0; j < 2; j++) {
            if (a.test(j)) {
                p[j] = box_max[j];
            } else {
                p[j] = box_min[j];
            }
        }
        points.push_back({{p[0], p[1]}});
    }

    const double voxel_resolution = diagonal_length / 20.0;
    std::array<int, 2> N; // number of grid points per dimension
    std::array<double, 2> h; // distance between grid points per dimension
    for (int i = 0; i < 2; i++) {
        const double D = box_max[i] - box_min[i];
        N[i] = (D / voxel_resolution) + 1;
        h[i] = D / N[i];
    }

    std::array<std::vector<double>, 2> ds;
    for (int i = 0; i < 2; i++) {
        ds[i].push_back(box_min[i]);
        for (int j = 0; j < N[i] - 1; j++) {
            ds[i].push_back(box_min[i] + h[i] * (j + 1));
        }
        ds[i].push_back(box_max[i]);
    }

    wmtk::SampleEnvelope envelope;
    // init envelope
    {
        std::vector<Vector2d> V_envelope;
        std::vector<Vector2i> E_envelope;
        for (int i = 0; i < E.rows(); i++) {
            E_envelope.push_back(Vector2i(E(i, 0), E(i, 1)));
        }
        for (int i = 0; i < V.rows(); i++) {
            V_envelope.push_back(V.row(i));
        }
        envelope.init(V_envelope, E_envelope, 0);
    }


    const double min_dis = voxel_resolution * voxel_resolution / 4;
    //    double min_dis = state.target_edge_len * state.target_edge_len;//epsilon*2
    for (int i = 0; i < ds[0].size(); i++) {
        for (int j = 0; j < ds[1].size(); j++) {
            if ((i == 0 || i == ds[0].size() - 1) && (j == 0 || j == ds[1].size() - 1)) {
                continue;
            }
            const Vector2d p(ds[0][i], ds[1][j]);

            Eigen::Vector2d n;
            const double sqd = envelope.nearest_point(p, n);

            if (sqd < min_dis) {
                continue;
            }
            points.push_back({{ds[0][i], ds[1][j]}});
        }
    }

    // CDT
    MatrixXd V_cdt;
    V_cdt.resize(points.size(), 2);
    for (int i = 0; i < points.size(); i++) {
        V_cdt.row(i) = Eigen::Vector2d(points[i][0], points[i][1]);
    }
    delaunay::constrained_delaunay2D(V_cdt, E, V_out, F_out, E_out);
}

void init_from_paths(
    const std::vector<std::string>& input_paths,
    MatrixXd& V_out,
    MatrixXi& F_out,
    MatrixXi& E_out)
{
    std::vector<MatrixXd> Vs;
    std::vector<MatrixXi> Es;
    // read input edge meshes
    for (const std::string& path : input_paths) {
        MatrixXd V;
        MatrixXi E;
        io::read_edge_mesh(path, V, E);
        logger().info("Read edge mesh {}: #V = {}, #E = {}", path, V.rows(), E.rows());
        V = V.block(0, 0, V.rows(), 2).eval(); // only keep x, y
        Vs.push_back(V);
        Es.push_back(E);
    }
    if (Vs.size() != Es.size()) {
        log_and_throw_error("Vs and Es size mismatch!");
    }

    // generate Delaunay triangulation of the input vertices as the initial mesh
    {
        MatrixXd V_all;
        MatrixXi E_all;
        std::vector<Eigen::Vector2d> V_vec;
        std::vector<Eigen::Vector2i> E_vec;
        for (const auto& V : Vs) {
            for (int i = 0; i < V.rows(); i++) {
                V_vec.push_back(V.row(i));
            }
        }
        for (const auto& E : Es) {
            for (int i = 0; i < E.rows(); i++) {
                E_vec.push_back(E.row(i));
            }
        }
        V_all.resize(V_vec.size(), 2);
        for (int i = 0; i < V_vec.size(); i++) {
            V_all.row(i) = V_vec[i];
        }
        E_all.resize(E_vec.size(), 2);
        for (int i = 0; i < E_vec.size(); i++) {
            E_all.row(i) = E_vec[i];
        }
        init_from_delaunay_box_mesh(V_all, E_all, V_out, F_out, E_out);
    }

    // logger().info("CDT mesh: #V = {}, #F = {}, #E = {}", V_out.rows(), F_out.rows(),
    // E_out.rows());
}

} // namespace wmtk::components::triwild