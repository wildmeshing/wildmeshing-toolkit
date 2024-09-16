#include "local_joint_flatten.hpp"
#include <igl/boundary_facets.h>
#include <igl/boundary_loop.h>
#include <igl/cotmatrix_entries.h>
#include <igl/doublearea.h>
#include <igl/slice.h>
#include <igl/slice_into.h>
#include <igl/triangle/scaf.h>

// flatten part is done
namespace wmtk::operations::utils {
void flatten(
    const Eigen::MatrixXd& V_joint_before,
    const Eigen::MatrixXd& V_joint_after,
    const Eigen::MatrixXi& F_joint_before,
    const Eigen::MatrixXi& F_joint_after,
    const Eigen::VectorXi& b_soft,
    Eigen::MatrixXd& UVjoint,
    int n_iterations)
{
    Eigen::MatrixXd bnd_uv, uv_init;
    Eigen::VectorXi bnd;

    igl::boundary_loop(F_joint_before, bnd);
    Eigen::MatrixXd M_before;
    igl::doublearea(V_joint_before, F_joint_before, M_before);

    igl::map_vertices_to_circle(V_joint_before, bnd, bnd_uv);
    bnd_uv *= sqrt(M_before.sum() / (2 * igl::PI));


    // add hard constraints
    std::vector<std::pair<int, int>> b_hard;
    if (b_soft.size() > 0) {
        for (int i = 0; i < b_soft.size(); i++) {
            b_hard.push_back(std::make_pair(b_soft(i), 0));
        }
        double fixed_value = 0;
        bool find = false;
        for (int i = 0; i < bnd.size(); i++) {
            for (int j = 0; j < b_soft.size(); j++)
                if (bnd[i] == b_soft[j]) {
                    if (find) {
                        bnd_uv(i, 0) = fixed_value;
                    } else {
                        find = true;
                        fixed_value = bnd_uv(i, 0);
                    }
                }
        }
    }

    Eigen::MatrixXi F_joint;
    // F_joint = [F_joint_before; F_joint_after];
    F_joint.resize(F_joint_before.rows() + F_joint_after.rows(), F_joint_before.cols());
    F_joint << F_joint_before, F_joint_after;

    // get uv_init
    igl::harmonic(V_joint_before, F_joint, bnd, bnd_uv, 1, uv_init);
    igl::triangle::SCAFData scaf_data;

    // optimization
    Eigen::VectorXi b; // place-holder
    Eigen::MatrixXd bc; // place-holder
    igl::triangle::scaf_precompute_joint(
        V_joint_before,
        V_joint_after,
        F_joint,
        F_joint_before,
        F_joint_after,
        uv_init,
        scaf_data,
        igl::MappingEnergyType::SYMMETRIC_DIRICHLET,
        b,
        bc,
        0,
        b_hard);
    igl::triangle::scaf_solve(scaf_data, n_iterations);

    // return UVjoint
    UVjoint = scaf_data.w_uv;
}

void get_local_vid_map(
    const std::vector<int>& v_id_map_before,
    const std::vector<int>& v_id_map_after,
    std::vector<int>& local_vid_after_to_before_map,
    int& vi_before,
    int& vj_before,
    int& vi_after)
{
    vi_after = 0;

    local_vid_after_to_before_map.resize(v_id_map_after.size(), -1);
    for (int i = 1; i < v_id_map_after.size(); i++) {
        auto it = std::find(v_id_map_before.begin(), v_id_map_before.end(), v_id_map_after[i]);
        if (it == v_id_map_before.end()) {
            std::runtime_error("Error: vertex not found in v_id_map_before");
        }
        local_vid_after_to_before_map[i] = std::distance(v_id_map_before.begin(), it);
    }
    vi_before = 0;
    vj_before = -1;
    for (int i = 0; i < v_id_map_before.size(); i++) {
        if (std::find(v_id_map_after.begin(), v_id_map_after.end(), v_id_map_before[i]) ==
            v_id_map_after.end()) {
            vj_before = i;
            break;
        }
    }

    std::cout << "vj_before = " << vj_before << std::endl;
    if (vj_before == -1 || vj_before == vi_before) {
        throw std::runtime_error("Cannot find the joint vertex!");
    }
}

void local_joint_flatten(
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXd& V_before,
    const std::vector<int64_t>& v_id_map_before,
    Eigen::MatrixXi& F_after,
    const Eigen::MatrixXd& V_after,
    const std::vector<int64_t>& v_id_map_after,
    Eigen::MatrixXd& UV_joint,
    std::vector<int64_t>& v_id_map_joint,
    bool is_bd_v0,
    bool is_bd_v1)
{
    int case_id = 0;
    if (is_bd_v0) case_id += 1;
    if (is_bd_v1) case_id += 1;
    std::cout << "Boundary case id: " << case_id << std::endl;

    // get joint mesh here
    Eigen::MatrixXd V_joint_before, V_joint_after;
    Eigen::MatrixXi F_joint_before, F_joint_after;


    // TODO: implement this part
    // flatten(V_joint_before, V_joint_after, F_joint_before, F_joint_after, b_soft, UV_joint, 10);

    // check if all element in UV_joint is valid number
    bool check_nan = false;
    for (int i = 0; i < UV_joint.rows(); i++) {
        for (int j = 0; j < UV_joint.cols(); j++) {
            if (std::isnan(UV_joint(i, j))) {
                std::cout << "nan discovered in UV_joint!" << std::endl;
                check_nan = true;
                break;
            }
        }
        if (check_nan) {
            break;
        }
    }

    if (check_nan) {
        std::runtime_error("There is nan in UV_joint!");
    }
}

void local_joint_flatten_smoothing(
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXd& V_after,
    Eigen::MatrixXi& F_after,
    Eigen::MatrixXd& UV_joint)
{
    return;
}

void local_joint_flatten_swap(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXi& F_after,
    Eigen::MatrixXd& UV_joint,
    Eigen::VectorXi& local_vid_after_to_before_map)
{
    return;
}

} // namespace wmtk::operations::utils