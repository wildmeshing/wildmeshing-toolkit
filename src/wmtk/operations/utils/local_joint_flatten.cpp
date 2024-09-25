#include "local_joint_flatten.hpp"
#include <wmtk/utils/orient.hpp>

#include <igl/boundary_facets.h>
#include <igl/boundary_loop.h>
#include <igl/cotmatrix_entries.h>
#include <igl/doublearea.h>
#include <igl/slice.h>
#include <igl/slice_into.h>
#include <igl/triangle/scaf.h>

// TODO: DEBUG
#include <igl/opengl/glfw/Viewer.h>


// flatten part is done
namespace wmtk::operations::utils {

void uniform_uv_init(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::VectorXi& bnd,
    const Eigen::MatrixXd& bnd_uv,
    Eigen::MatrixXd& uv_init)
{
    uv_init.resize(V.rows(), 2);

    int uv_size = V.rows();
    int bnd_size = bnd.size();

    std::vector<int> is_bnd(uv_size, -1);
    for (int i = 0; i < bnd.size(); i++) {
        is_bnd[bnd[i]] = i;
    }
    Eigen::MatrixXd A(uv_size, bnd_size);
    A.setZero();
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            int v0 = F(i, j);
            int v1 = F(i, (j + 1) % 3);
            if (is_bnd[v0] >= 0) {
                A(v1, is_bnd[v0]) = 1;
            }

            if (is_bnd[v1] >= 0) {
                A(v0, is_bnd[v1]) = 1;
            }
        }
    }

    for (int i = 0; i < uv_size; i++) {
        if (is_bnd[i] >= 0) {
            uv_init.row(i) = bnd_uv.row(is_bnd[i]);
        } else {
            uv_init.row(i) = (A.row(i) * bnd_uv) / A.row(i).sum();
            if (A.row(i).sum() == 2) {
                uv_init.row(i) *= 0.9;
            }
        }
    }
}

void flatten(
    const Eigen::MatrixXd& V_joint_before,
    const Eigen::MatrixXd& V_joint_after,
    const Eigen::MatrixXi& F_joint_before,
    const Eigen::MatrixXi& F_joint_after,
    // const Eigen::VectorXi& b_soft,
    const std::vector<std::pair<int, int>>& b_hard,
    Eigen::MatrixXd& UVjoint,
    int n_iterations,
    bool debug_mode = false)
{
    Eigen::MatrixXd bnd_uv, uv_init;
    Eigen::VectorXi bnd;

    igl::boundary_loop(F_joint_before, bnd);
    Eigen::MatrixXd M_before;
    igl::doublearea(V_joint_before, F_joint_before, M_before);

    // may causing problem?
    // igl::map_vertices_to_circle(V_joint_before, bnd, bnd_uv);
    // Here we change to uniform circle
    
    // TODO: Modify the colinear case
    // normally we use starting as 0
    {
        int starting = 0;
        if (b_hard.size() > 0){
            // find b_hard[1].first in bnd
            for (int i = 0; i < bnd.size(); i++){
                if (b_hard[1].first == bnd[i]){
                    starting = i;
                    break;
                }
            }
        }
        bnd_uv.resize(bnd.size(), 2);
        for (int i = 0; i < bnd.size(); i++) {
            bnd_uv((starting + i) % bnd.size(), 0) = cos(2 * igl::PI * i / bnd.size());
            bnd_uv((starting + i) % bnd.size(), 1) = sin(2 * igl::PI * i / bnd.size());
        }
    }
    bnd_uv *= sqrt(M_before.sum() / (2 * igl::PI));

    // add hard constraints
    if (b_hard.size() > 0) {
        double fixed_value = 0;
        bool find = false;
        for (int i = 0; i < bnd.size(); i++) {
            for (int j = 0; j < b_hard.size(); j++)
                if (bnd[i] == b_hard[j].first) {
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

    auto check_uv_orientation = [](const Eigen::MatrixXd& uv, const Eigen::MatrixXi& F) {
        for (int i = 0; i < F.rows(); i++) {
            if (wmtk::utils::wmtk_orient2d(uv.row(F(i, 0)), uv.row(F(i, 1)), uv.row(F(i, 2))) < 0) {
                return false;
            }
        }
        return true;
    };

    // get uv_init
    igl::harmonic(V_joint_before, F_joint, bnd, bnd_uv, 1, uv_init);
    // TODO: 1. first, check orientation
    //       2. then, if fail, find another way to get uv_init
    {
        if (!check_uv_orientation(uv_init, F_joint)) {
            std::cout << "Use uniform uv init!" << std::endl;
            uniform_uv_init(V_joint_before, F_joint, bnd, bnd_uv, uv_init);
            if (!check_uv_orientation(uv_init, F_joint)) {
                std::runtime_error("Orientation check failed for uniform uv init!");
            }
        }
    }
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
        // igl::MappingEnergyType::ARAP,
        igl::MappingEnergyType::SYMMETRIC_DIRICHLET,
        // igl::MappingEnergyType::CONFORMAL,
        b,
        bc,
        0,
        b_hard);


    if (debug_mode) {
        int show_option = 1;
        auto key_down_debug = [&](igl::opengl::glfw::Viewer& viewer,
                                  unsigned char key,
                                  int modifier) {
            if (key == '1')
                show_option = 1;
            else if (key == '2')
                show_option = 2;
            else if (key == '3')
                show_option = 3;
            else if (key == '4')
                show_option = 4;

            Eigen::MatrixXi F_all;
            Eigen::MatrixXd shifted_uv;
            shifted_uv = scaf_data.w_uv;
            double max_x_axis = shifted_uv.col(0).maxCoeff();
            double min_x_axis = shifted_uv.col(0).minCoeff();
            shifted_uv.col(0).array() += 1.1 * (max_x_axis - min_x_axis);
            Eigen::MatrixXd uv_all(scaf_data.w_uv.rows() * 2, scaf_data.w_uv.cols());
            uv_all << scaf_data.w_uv, shifted_uv;

            if (key == ' ') {
                std::cout << "start solving scaf" << std::endl;
                igl::triangle::scaf_solve(scaf_data, 1);
                std::cout << "finish solving scaf for one iteration" << std::endl;
                shifted_uv = scaf_data.w_uv;
                max_x_axis = shifted_uv.col(0).maxCoeff();
                min_x_axis = shifted_uv.col(0).minCoeff();
                shifted_uv.col(0).array() += 1.1 * (max_x_axis - min_x_axis);
                uv_all.resize(scaf_data.w_uv.rows() * 2, scaf_data.w_uv.cols());
                uv_all << scaf_data.w_uv, shifted_uv;
            }

            // get max y coordinate of scaf_data.w_uv

            switch (show_option) {
            case 2: {
                viewer.data().clear();
                viewer.data().set_mesh(V_joint_after, F_joint_after);
                viewer.data().set_colors(Eigen::RowVector3d(230, 220, 170) / 255.0);
                viewer.core().align_camera_center(V_joint_after, F_joint_after);
                break;
            }
            case 3: {
                viewer.data().clear();
                Eigen::MatrixXi F_w_before, F_w_after;
                Eigen::MatrixXd C;
                igl::cat(1, F_joint_before, scaf_data.s_T, F_w_before);
                igl::cat(1, F_joint_after, scaf_data.s_T, F_w_after);
                C.resize(F_w_before.rows() + F_w_after.rows(), 3);
                for (int i = 0; i < F_joint_before.rows(); i++) {
                    C.row(i) = Eigen::RowVector3d(230, 220, 170) / 255.0;
                }
                for (int i = 0; i < scaf_data.s_T.rows(); i++) {
                    C.row(i + F_joint_before.rows()) = Eigen::RowVector3d(210, 150, 150) / 255.0;
                }
                for (int i = 0; i < F_joint_after.rows(); i++) {
                    C.row(i + F_w_before.rows()) = Eigen::RowVector3d(230, 220, 170) / 255.0;
                }
                for (int i = 0; i < scaf_data.s_T.rows(); i++) {
                    C.row(i + F_w_before.rows() + F_joint_after.rows()) =
                        Eigen::RowVector3d(210, 150, 150) / 255.0;
                }
                F_all.resize(F_w_before.rows() + F_w_after.rows(), F_w_before.cols());
                F_all << F_w_before, (F_w_after.array() + scaf_data.w_uv.rows());
                viewer.data().set_mesh(uv_all, F_all);
                viewer.data().set_colors(C);
                viewer.core().align_camera_center(uv_init, F_joint_before);

                break;
            }
            case 1: {
                viewer.data().clear();
                viewer.data().set_mesh(V_joint_before, F_joint_before);
                viewer.data().set_colors(Eigen::RowVector3d(230, 220, 170) / 255.0);
                viewer.core().align_camera_center(V_joint_before, F_joint_before);

                break;
            }
            }
            return false;
        };

        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(V_joint_before, F_joint_before);
        viewer.data().set_colors(Eigen::RowVector3d(230, 220, 170) / 255.0);
        viewer.callback_key_down = key_down_debug;
        viewer.launch();
    } else {
        igl::triangle::scaf_solve(scaf_data, n_iterations);
    }
    // return UVjoint
    UVjoint = scaf_data.w_uv.topRows(V_joint_before.rows());
}

// get local_vid_map from V_joint_after to V_joint_before
void get_local_vid_map(
    const std::vector<int64_t>& v_id_map_before,
    const std::vector<int64_t>& v_id_map_after,
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

    // std::cout << "vj_before = " << vj_before << std::endl;
    if (vj_before == -1 || vj_before == vi_before) {
        throw std::runtime_error("Cannot find the joint vertex!");
    }
}


// input: v_id_map_before, v_id_map_after
// input: is_bd_v0, is_bd_v1
// output: V_joint_before, V_joint_after, F_joint_before, F_joint_after
// output: b_hard(hard constraint on colinear case on the boundary)
int get_joint_mesh(
    const Eigen::MatrixXd& V_before,
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXd& V_after,
    const Eigen::MatrixXi& F_after,
    const std::vector<int64_t>& v_id_map_before,
    const std::vector<int64_t>& v_id_map_after,
    bool is_bd_v0,
    bool is_bd_v1,
    Eigen::MatrixXd& V_joint_before,
    Eigen::MatrixXi& F_joint_before,
    Eigen::MatrixXd& V_joint_after,
    Eigen::MatrixXi& F_joint_after,
    std::vector<std::pair<int, int>>& b_hard,
    std::vector<int64_t>& v_id_map_joint)
{
    // get boundary_case_id
    int case_id = 0;
    if (is_bd_v0) case_id += 1;
    if (is_bd_v1) case_id += 1;
    std::cout << "Boundary case id: " << case_id << std::endl;

    // collect information
    int vi_before, vj_before, vi_after;
    std::vector<int> local_vid_after_to_before_map;
    get_local_vid_map(
        v_id_map_before,
        v_id_map_after,
        local_vid_after_to_before_map,
        vi_before,
        vj_before,
        vi_after);

    if (case_id == 0) {
        // for collapse operation there is one more vertex in the after mesh
        int N_v_joint = V_before.rows() + 1;

        // build V_joint_before, and V_joint_after
        V_joint_before = V_before;
        V_joint_before.conservativeResize(N_v_joint, V_before.cols());
        V_joint_before.row(V_before.rows()) = V_after.row(vi_after);

        V_joint_after = V_joint_before;

        // joint the two meshes
        // get F_joint,(first after, then before)
        F_joint_before = F_before;
        F_joint_after.resize(F_after.rows(), F_after.cols());
        // build F_joint_after
        {
            local_vid_after_to_before_map[vi_after] = N_v_joint - 1;
            for (int i = 0; i < F_joint_after.rows(); i++) {
                for (int j = 0; j < F_joint_after.cols(); j++) {
                    F_joint_after(i, j) = local_vid_after_to_before_map[F_after(i, j)];
                }
            }
        }

        v_id_map_joint = v_id_map_before;
        v_id_map_joint.push_back(v_id_map_after[vi_after]);

        return N_v_joint - 1;

    } else if (case_id == 1) {
        // for connector case there will be no more vertices than before case
        int N_v_joint = V_before.rows();

        // build V_joint_before, and V_joint_after
        V_joint_before = V_before;
        V_joint_after = V_joint_before;
        // which vertex is on the boundary
        // note that vj is the one to be kept
        int v_bd = is_bd_v0 ? vj_before : vi_before;
        V_joint_after.row(v_bd) = V_after.row(vi_after);

        // joint the two meshes
        // get F_joint,(first after, then before)
        F_joint_before = F_before;
        F_joint_after.resize(F_after.rows(), F_after.cols());
        // build F_joint_after
        {
            local_vid_after_to_before_map[vi_after] =
                v_bd; // Note this line is different from case 0
            for (int i = 0; i < F_joint_after.rows(); i++) {
                for (int j = 0; j < F_joint_after.cols(); j++) {
                    F_joint_after(i, j) = local_vid_after_to_before_map[F_after(i, j)];
                }
            }
        }

        v_id_map_joint = v_id_map_before;
        v_id_map_joint.push_back(v_id_map_after[vi_after]);

        return v_bd;

    } else if (case_id == 2) {
        bool do_3_colinear_case = true;
        int v_to_keep;

        // decide which vertex to keep
        {
            Eigen::VectorXi bd_loop_before;
            igl::boundary_loop(F_before, bd_loop_before);

            int i_idx = std::distance(
                bd_loop_before.begin(),
                std::find(bd_loop_before.begin(), bd_loop_before.end(), vi_before));
            int j_idx = std::distance(
                bd_loop_before.begin(),
                std::find(bd_loop_before.begin(), bd_loop_before.end(), vj_before));
            int offset = 1;
            if (bd_loop_before[i_idx + offset] == vj_before) {
                offset = -1;
            }
            // DEBUG CHECK
            if (bd_loop_before[(i_idx + bd_loop_before.size() - offset) % bd_loop_before.size()] !=
                vj_before) {
                std::runtime_error("Something wrong with the boundary loop in 3-colinear method");
            }
            // vp, vi, vj, vq in order
            int v_p =
                bd_loop_before[(i_idx + bd_loop_before.size() + offset) % bd_loop_before.size()];
            int v_q =
                bd_loop_before[(j_idx + bd_loop_before.size() - offset) % bd_loop_before.size()];

            auto in_same_triangle = [&](int v0, int v2, int v3) {
                for (int i = 0; i < F_before.rows(); i++) {
                    if ((F_before(i, 0) == v0 || F_before(i, 1) == v0 || F_before(i, 2) == v0) &&
                        (F_before(i, 0) == v2 || F_before(i, 1) == v2 || F_before(i, 2) == v2) &&
                        (F_before(i, 0) == v3 || F_before(i, 1) == v3 || F_before(i, 2) == v3)) {
                        return true;
                    }
                }
                return false;
            };

            b_hard.resize(3);

            if (in_same_triangle(vi_before, vj_before, v_q)) {
                v_to_keep = vj_before;
                // keep vj means vp, vi, vj colinear
                b_hard.resize(3);
                b_hard[0] = std::make_pair(v_p, 0);
                b_hard[1] = std::make_pair(vi_before, 0);
                b_hard[2] = std::make_pair(vj_before, 0);
            } else {
                v_to_keep = vi_before;
                // keep vi means vp, vi, vq colinear
                b_hard.resize(3);
                b_hard[0] = std::make_pair(vi_before, 0);
                b_hard[1] = std::make_pair(vj_before, 0);
                b_hard[2] = std::make_pair(v_q, 0);
            }
        }

        if (do_3_colinear_case) {
            // for connector case there will be no more vertices than before case
            int N_v_joint = V_before.rows();

            // build V_joint_before, and V_joint_after
            V_joint_before = V_before;
            V_joint_after = V_joint_before;
            V_joint_after.row(v_to_keep) = V_after.row(vi_after);

            // joint the two meshes
            // get F_joint,(first after, then before)
            F_joint_before = F_before;
            F_joint_after.resize(F_after.rows(), F_after.cols());
            // build F_joint_after
            {
                local_vid_after_to_before_map[vi_after] =
                    v_to_keep; // Note this line is different from case 0
                for (int i = 0; i < F_joint_after.rows(); i++) {
                    for (int j = 0; j < F_joint_after.cols(); j++) {
                        F_joint_after(i, j) = local_vid_after_to_before_map[F_after(i, j)];
                    }
                }
            }

            v_id_map_joint = v_id_map_before;
            v_id_map_joint.push_back(v_id_map_after[vi_after]);

            return v_to_keep;
        } // end of if(do_3_colinear_case)
    }

    return -1; // should not happen
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
    bool is_bd_v1,
    bool debug_mode)
{
    // get joint mesh here
    Eigen::MatrixXd V_joint_before, V_joint_after;
    Eigen::MatrixXi F_joint_before, F_joint_after;

    std::vector<std::pair<int, int>> b_hard;

    int v_common = get_joint_mesh(
        V_before,
        F_before,
        V_after,
        F_after,
        v_id_map_before,
        v_id_map_after,
        is_bd_v0,
        is_bd_v1,
        V_joint_before,
        F_joint_before,
        V_joint_after,
        F_joint_after,
        b_hard,
        v_id_map_joint);

    flatten(
        V_joint_before,
        V_joint_after,
        F_joint_before,
        F_joint_after,
        b_hard,
        UV_joint,
        5,
        debug_mode);

    // modify UV_joint, F_after
    {
        // only add the common vertex for case 1 and case 2
        // here the if condition means no vertices are added
        if (v_common != V_before.rows()) {
            UV_joint.conservativeResize(UV_joint.rows() + 1, UV_joint.cols());
            UV_joint.row(UV_joint.rows() - 1) = UV_joint.row(v_common);
        }
        F_after = F_joint_after;

        for (int i = 0; i < F_after.rows(); i++) {
            for (int j = 0; j < 3; j++) {
                if (F_after(i, j) == v_common) {
                    F_after(i, j) = UV_joint.rows() - 1;
                }
            }
        }
    }

    /*
        {
            int show_option = 1;
            auto key_down_init =
                [&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier) {
                    if (key == '1')
                        show_option = 1;
                    else if (key == '2')
                        show_option = 2;
                    else if (key == '3')
                        show_option = 3;
                    else if (key == '4')
                        show_option = 4;


                    switch (show_option) {
                    case 1:
                        viewer.data().clear();
                        viewer.data().set_mesh(V_joint_before, F_joint_before);
                        viewer.core().align_camera_center(V_joint_before, F_joint_before);
                        break;
                    case 2:
                        viewer.data().clear();
                        viewer.data().set_mesh(V_joint_after, F_joint_after);
                        viewer.core().align_camera_center(V_joint_after, F_joint_after);
                        break;
                    case 3:
                        viewer.data().clear();
                        viewer.data().set_mesh(UV_joint, F_joint_before);
                        viewer.data().set_colors(Eigen::RowVector3d(210, 150, 150) / 255.0);
                        viewer.core().align_camera_center(UV_joint, F_joint_before);
                        break;
                    case 4:
                        viewer.data().clear();
                        viewer.data().set_mesh(UV_joint, F_joint_after);
                        viewer.data().set_colors(Eigen::RowVector3d(210, 150, 150) / 255.0);
                        viewer.core().align_camera_center(UV_joint, F_joint_after);
                        break;
                    }

                    return false;
                };
            igl::opengl::glfw::Viewer viewer;
            viewer.data().set_mesh(V_joint_before, F_joint_before);
            viewer.callback_key_down = key_down_init;
            viewer.launch();
        }
        */

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
    Eigen::MatrixXd& UV_joint,
    bool debug_mode)
{

    // TODO: special case is needed for boundary cases

    // in this case V_joint_before == V_joint_after
    Eigen::MatrixXd V_joint = V_before;
    V_joint.conservativeResize(V_joint.rows() + 1, V_joint.cols());
    V_joint.row(V_joint.rows() - 1) = V_after.row(0);

    Eigen::MatrixXi F_joint_before = F_before;
    Eigen::MatrixXi F_joint_after = F_before;

    // update F_joint_after
    for (int i = 0; i < F_joint_after.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            if (F_joint_after(i, j) == 0) {
                F_joint_after(i, j) = V_joint.rows() - 1;
            }
        }
    }

    F_after = F_joint_after;
    std::vector<std::pair<int, int>> b_hard; // empty

    std::cout << "Start flatten smoothing" << std::endl;
    flatten(V_joint, V_joint, F_joint_before, F_joint_after, b_hard, UV_joint, 8, debug_mode);
    std::cout << "Finish flatten smoothing" << std::endl;
    /*
    {
        auto V_joint_before = V_joint;
        auto V_joint_after = V_joint;
        int show_option = 1;
        auto key_down_init =
            [&](igl::opengl::glfw::Viewer& viewer, unsigned char key, int modifier) {
                if (key == '1')
                    show_option = 1;
                else if (key == '2')
                    show_option = 2;
                else if (key == '3')
                    show_option = 3;
                else if (key == '4')
                    show_option = 4;


                switch (show_option) {
                case 1:
                    viewer.data().clear();
                    viewer.data().set_mesh(V_joint_before, F_joint_before);
                    viewer.core().align_camera_center(V_joint_before, F_joint_before);
                    break;
                case 2:
                    viewer.data().clear();
                    viewer.data().set_mesh(V_joint_after, F_joint_after);
                    viewer.core().align_camera_center(V_joint_after, F_joint_after);
                    break;
                case 3:
                    viewer.data().clear();
                    viewer.data().set_mesh(UV_joint, F_joint_before);
                    viewer.data().set_colors(Eigen::RowVector3d(210, 150, 150) / 255.0);
                    viewer.core().align_camera_center(UV_joint, F_joint_before);
                    break;
                case 4:
                    viewer.data().clear();
                    viewer.data().set_mesh(UV_joint, F_joint_after);
                    viewer.data().set_colors(Eigen::RowVector3d(210, 150, 150) / 255.0);
                    viewer.core().align_camera_center(UV_joint, F_joint_after);
                    break;
                }

                return false;
            };
        igl::opengl::glfw::Viewer viewer;
        viewer.data().set_mesh(V_joint_before, F_joint_before);
        viewer.callback_key_down = key_down_init;
        viewer.launch();
    }
    */
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
    // get local_vid_after_to_before_map
    local_vid_after_to_before_map.resize(V_after.rows());
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            if (V_before.row(i) == V_after.row(j)) {
                local_vid_after_to_before_map[j] = i;
                break;
            }
        }
    }

    Eigen::MatrixXi F_after_joint;
    F_after_joint.resize(F_after.rows(), F_after.cols());
    for (int i = 0; i < F_after.rows(); i++) {
        for (int j = 0; j < F_after.cols(); j++) {
            F_after_joint(i, j) = local_vid_after_to_before_map[F_after(i, j)];
        }
    }

    std::vector<std::pair<int, int>> b_hard; // empty

    std::cout << "Start flatten swapping" << std::endl;
    flatten(V_before, V_before, F_before, F_after_joint, b_hard, UV_joint, 5);
    std::cout << "Finish flatten swapping" << std::endl;


    return;
}

} // namespace wmtk::operations::utils