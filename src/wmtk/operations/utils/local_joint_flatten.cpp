#include "local_joint_flatten.hpp"
#include <igl/boundary_facets.h>
#include <igl/boundary_loop.h>
#include <igl/cotmatrix_entries.h>
#include <igl/doublearea.h>
#include <igl/slice.h>
#include <igl/slice_into.h>
// min_quad_with_fixed precomputation
struct mqwf_dense_data
{
    int n;
    Eigen::VectorXi k; // known
    Eigen::VectorXi u; // unknown
    Eigen::LDLT<Eigen::MatrixXd> Auu_pref;
    Eigen::MatrixXd Auk_plus_AkuT;
};
void mqwf_dense_precompute(
    const Eigen::MatrixXd& A,
    const Eigen::MatrixXi& known,
    mqwf_dense_data& data)
{
    using namespace std;
    using namespace Eigen;

    // get number of variables
    int n = A.rows();

    // set know indices
    data.k.resize(known.size());
    data.k = known;

    // get unknown indices
    data.u.resize(n - data.k.size());
    {
        VectorXi known_sort;
        igl::sort(known, 1, true, known_sort);
        VectorXi tmp = VectorXi::LinSpaced(n, 0, n - 1);
        auto it = std::set_difference(
            tmp.data(),
            tmp.data() + tmp.size(),
            known_sort.data(),
            known_sort.data() + known_sort.size(),
            data.u.data());
        data.u.conservativeResize(std::distance(data.u.data(), it));
    }
    assert((data.u.size() + data.k.size()) == n);

    // get matrices
    MatrixXd Auu, Auk, Aku;
    Auu.resize(data.u.size(), data.u.size());
    igl::slice(A, data.u, data.u, Auu);
    igl::slice(A, data.u, data.k, Auk);
    igl::slice(A, data.k, data.u, Aku);

    // save data
    data.Auu_pref = Auu.ldlt();
    // data.Auu_pref = Auu.llt();
    data.Auk_plus_AkuT = Auk + Aku.transpose();
    data.n = n;
};
void mqwf_dense_solve(
    const mqwf_dense_data& data,
    const Eigen::VectorXd& RHS,
    const Eigen::VectorXd& known_val,
    Eigen::VectorXd& sol)
{
    using namespace std;
    using namespace Eigen;

    // get column : in order to do B = A(i,:)
    VectorXi col_colon = VectorXi::LinSpaced(RHS.cols(), 0, RHS.cols() - 1);

    // construct the reduced system
    // data.Auu * sol = RHS_reduced
    VectorXd RHS_reduced;
    if (data.k.size() == 0) // no know values
        RHS_reduced = -RHS;
    else {
        VectorXd RHS_unknown;
        igl::slice(RHS, data.u, col_colon, RHS_unknown);
        RHS_reduced = -0.5 * data.Auk_plus_AkuT * known_val - RHS_unknown;
    }

    // solve
    sol.resize(data.n, RHS.cols());

    VectorXd sol_unknown;
    sol_unknown.resize(data.u.size(), RHS.cols());
    sol_unknown = data.Auu_pref.solve(RHS_reduced);
    igl::slice_into(sol_unknown, data.u, col_colon, sol); // sol(unknown,:) = sol_unknown
    igl::slice_into(known_val, data.k, col_colon, sol); // sol(known,:) = known_val
}

// matrix assembly helpers
void vector_area_matrix_size(const Eigen::MatrixXi& F, const int nV, Eigen::MatrixXd& A)
{
    using namespace Eigen;
    using namespace std;

    // number of vertices
    MatrixXi E;
    igl::boundary_facets(F, E);

    A.resize(nV * 2, nV * 2);
    A.setZero();

    for (int k = 0; k < E.rows(); k++) {
        int i = E(k, 0);
        int j = E(k, 1);
        A(i + nV, j) -= 0.25;
        A(j, i + nV) -= 0.25;
        A(i, j + nV) += 0.25;
        A(j + nV, i) += 0.25;
    }
}
void cotmatrix_dense(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, Eigen::MatrixXd& A)
{
    using namespace Eigen;

    // dense cotmatrix
    MatrixXd C;
    igl::cotmatrix_entries(V, F, C);

    // get edge
    MatrixXi edges;
    edges.resize(3, 2);
    edges << 1, 2, 2, 0, 0, 1;

    // init dense cotmatrix A
    A.resize(V.rows(), V.rows());
    A.setZero();

    // Loop over triangles
    for (int ii = 0; ii < F.rows(); ii++) {
        // loop over edges of element
        for (int e = 0; e < edges.rows(); e++) {
            int source = F(ii, edges(e, 0));
            int dest = F(ii, edges(e, 1));
            A(source, dest) += C(ii, e);
            A(dest, source) += C(ii, e);
            A(source, source) -= C(ii, e);
            A(dest, dest) -= C(ii, e);
        }
    }
}

// energy evaluation
// reference: "Texture Mapping Progressive Meshes"
void quasi_conformal_energy(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXd& uv, // UV
    Eigen::VectorXd& E)
{
    using namespace Eigen;

    E.resize(F.rows());
    for (int ff = 0; ff < F.rows(); ff++) {
        double s1 = uv(F(ff, 0), 0);
        double s2 = uv(F(ff, 1), 0);
        double s3 = uv(F(ff, 2), 0);

        double t1 = uv(F(ff, 0), 1);
        double t2 = uv(F(ff, 1), 1);
        double t3 = uv(F(ff, 2), 1);

        VectorXd q1 = V.row(F(ff, 0));
        VectorXd q2 = V.row(F(ff, 1));
        VectorXd q3 = V.row(F(ff, 2));

        std::cout << "q1: " << q1 << std::endl;
        std::cout << "q2: " << q2 << std::endl;
        std::cout << "q3: " << q3 << std::endl;

        std::cout << "t1: " << t1 << std::endl;
        std::cout << "t2: " << t2 << std::endl;
        std::cout << "t3: " << t3 << std::endl;

        std::cout << "s1: " << s1 << std::endl;
        std::cout << "s2: " << s2 << std::endl;
        std::cout << "s3: " << s3 << std::endl;

        double A = ((s2 - s1) * (t3 - t1) - (s3 - s1) * (t2 - t1)) / 2;

        std::cout << "A: " << A << std::endl;
        VectorXd Ss = (q1 * (t2 - t3) + q2 * (t3 - t1) + q3 * (t1 - t2)) / (2 * A);
        VectorXd St = (q1 * (s3 - s2) + q2 * (s1 - s3) + q3 * (s2 - s1)) / (2 * A);

        double a = Ss.transpose() * Ss;
        double b = Ss.transpose() * St;
        double c = St.transpose() * St;

        double sigma = sqrt((a + c + sqrt((a - c) * (a - c) + 4 * b * b)) / 2);
        double gamma = sqrt((a + c - sqrt((a - c) * (a - c) + 4 * b * b)) / 2);
        gamma = 1;
        std::cout << "a: " << a << std::endl;
        std::cout << "b: " << b << std::endl;
        std::cout << "c: " << c << std::endl;
        std::cout << "sigma: " << sigma << std::endl;
        std::cout << "gamma: " << gamma << std::endl;
        E(ff) = sigma / gamma;
    }
}


namespace wmtk::operations::utils {
void flatten(
    const Eigen::MatrixXd& Vjoint_before,
    const Eigen::MatrixXd& Vjoint_after,
    const Eigen::MatrixXi& Fjoint_before,
    const Eigen::MatrixXi& Fjoint_after,
    const Eigen::VectorXi& b_UV,
    const Eigen::VectorXd& bc_UV,
    Eigen::MatrixXd& UVjoint)
{
    using namespace Eigen;

    int nVjoint = Vjoint_before.rows();
    MatrixXd Q;
    {
        // get lscm matrix
        MatrixXd A_before, A_after;
        // Assemble the area matrix (note that A is #Vx2 by #Vx2)
        vector_area_matrix_size(Fjoint_before, nVjoint, A_before);
        vector_area_matrix_size(Fjoint_after, nVjoint, A_after);

        // Assemble the cotan laplacian matrix for UV
        MatrixXd L_before, L_after, LUV_before, LUV_after;
        cotmatrix_dense(Vjoint_before, Fjoint_before, L_before);
        cotmatrix_dense(Vjoint_after, Fjoint_after, L_after);

        // LUV_before = [L_before, 0; 0, L_before]
        LUV_before.resize(2 * L_before.rows(), 2 * L_before.cols());
        LUV_before.setZero();
        LUV_before.block(0, 0, L_before.rows(), L_before.cols()) = L_before;
        LUV_before.block(L_before.rows(), L_before.cols(), L_before.rows(), L_before.cols()) =
            L_before;

        // LUV_after = [L_after, 0; 0, L_after]
        LUV_after.resize(2 * L_after.rows(), 2 * L_after.cols());
        LUV_after.setZero();
        LUV_after.block(0, 0, L_after.rows(), L_after.cols()) = L_after;
        LUV_after.block(L_after.rows(), L_after.cols(), L_after.rows(), L_after.cols()) = L_after;

        // get matrix for quadratics
        Q = -LUV_before + 2. * A_before - LUV_after + 2. * A_after;
    }
    // Flatten the joint mesh UVs
    VectorXd UVjoint_flat;


    // solve UV jointly
    {
        // using derek's solver
        const VectorXd RHS = VectorXd::Zero(nVjoint * 2);
        mqwf_dense_data data;
        mqwf_dense_precompute(Q, b_UV, data);
        mqwf_dense_solve(data, RHS, bc_UV, UVjoint_flat);
    }

    // get UVjoint
    UVjoint.resize(nVjoint, 2);
    for (unsigned i = 0; i < UVjoint.cols(); ++i)
        UVjoint.col(UVjoint.cols() - i - 1) =
            UVjoint_flat.block(UVjoint.rows() * i, 0, UVjoint.rows(), 1);
}


// interior edge
void local_joint_flatten_case0(
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXd& V_before,
    const std::vector<int64_t>& v_id_map_before,
    Eigen::MatrixXi& F_after,
    const Eigen::MatrixXd& V_after,
    const std::vector<int64_t>& v_id_map_after,
    Eigen::MatrixXd& UV_joint,
    std::vector<int64_t>& v_id_map_joint)
{
    // get V_joint, F_joint_before, F_joint_after

    // TODO: this could be easier if we get local mesh from a "good" order
    int vi_after = 0;
    std::vector<int> local_vid_after_to_before_map(v_id_map_after.size(), -1);

    for (int i = 1; i < v_id_map_after.size(); i++) {
        auto it = std::find(v_id_map_before.begin(), v_id_map_before.end(), v_id_map_after[i]);
        if (it == v_id_map_before.end()) {
            std::runtime_error("There is vertex that is unituq in before!");
        }
        local_vid_after_to_before_map[i] = std::distance(v_id_map_before.begin(), it);
    }


    int vi_before = 0, vj_before = -1;
    for (int i = 0; i < v_id_map_before.size(); i++) {
        if (std::find(v_id_map_after.begin(), v_id_map_after.end(), v_id_map_before[i]) ==
            v_id_map_after.end()) {
            vj_before = i;
            break;
        }
    }

    // std::cout << "vi_after: " << vi_after << std::endl;
    // std::cout << "vi_before: " << vi_before << std::endl;
    // std::cout << "vj_before: " << vj_before << std::endl;

    if (vj_before == -1 || vj_before == vi_before) {
        throw std::runtime_error("Cannot find the joint vertex!");
    }

    Eigen::MatrixXd V_joint = V_before;
    V_joint.conservativeResize(V_joint.rows() + 1, V_joint.cols());
    // put vi_after to the end
    V_joint.row(V_joint.rows() - 1) = V_after.row(vi_after);

    Eigen::MatrixXi F_joint_before = F_before;
    Eigen::MatrixXi F_joint_after = F_after;
    // update F_joint after
    for (int i = 0; i < F_joint_after.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            if (F_joint_after(i, j) == vi_after) {
                F_joint_after(i, j) = V_joint.rows() - 1;
            } else {
                F_joint_after(i, j) = local_vid_after_to_before_map[F_joint_after(i, j)];
            }
        }
    }

    // modify F_after and v_id_map_joint
    F_after = F_joint_after;
    v_id_map_joint = v_id_map_before;
    v_id_map_joint.push_back(v_id_map_after[vi_after]);

    // bc
    Eigen::VectorXi b_UV;
    Eigen::VectorXd bc_UV;

    b_UV.resize(2 * 2, 1);
    bc_UV.resize(2 * 2, 1);
    int nVjoint = V_joint.rows();

    b_UV << vi_before, vj_before, vi_before + nVjoint, vj_before + nVjoint;
    bc_UV << 0, 1, 0, 0;

    // b_UV.resize(3 * 1, 1);
    // bc_UV.resize(3 * 1, 1);
    // int nVjoint = V_joint.rows();
    // b_UV << vi_before, vi_before + nVjoint, vj_before;
    // bc_UV << 0, 0, 1;

    // flatten
    flatten(V_joint, V_joint, F_joint_before, F_joint_after, b_UV, bc_UV, UV_joint);
}

// case1 edge connect a interior vertex and a boundary vertex
void local_joint_flatten_case1(
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXd& V_before,
    const std::vector<int64_t>& v_id_map_before,
    Eigen::MatrixXi& F_after,
    const Eigen::MatrixXd& V_after,
    const std::vector<int64_t>& v_id_map_after,
    Eigen::MatrixXd& UV_joint,
    std::vector<int64_t>& v_id_map_joint,
    bool is_bd_vj,
    bool is_bd_vi)
{
    // get V_joint_before, F_joint_before, V_joint_after, F_joint_after

    // TODO: this could be easier if we get local mesh from a "good" order
    // this part is the same as case0
    int vi_after = 0;
    std::vector<int> local_vid_after_to_before_map(v_id_map_after.size(), -1);

    for (int i = 1; i < v_id_map_after.size(); i++) {
        auto it = std::find(v_id_map_before.begin(), v_id_map_before.end(), v_id_map_after[i]);
        if (it == v_id_map_before.end()) {
            std::runtime_error("There is vertex that is unituq in before!");
        }
        local_vid_after_to_before_map[i] = std::distance(v_id_map_before.begin(), it);
    }
    int vi_before = 0, vj_before = -1;
    for (int i = 0; i < v_id_map_before.size(); i++) {
        if (std::find(v_id_map_after.begin(), v_id_map_after.end(), v_id_map_before[i]) ==
            v_id_map_after.end()) {
            vj_before = i;
            break;
        }
    }
    if (vj_before == -1 || vj_before == vi_before) {
        throw std::runtime_error("Cannot find the joint vertex!");
    }

    Eigen::MatrixXd V_joint_before = V_before;
    Eigen::MatrixXd V_joint_after = V_joint_before;

    int v_bd = is_bd_vi ? vi_before : vj_before;
    V_joint_after.row(v_bd) = V_after.row(vi_after);

    Eigen::MatrixXi F_joint_before = F_before;
    Eigen::MatrixXi F_joint_after = F_after;
    // update F_joint after
    for (int i = 0; i < F_joint_after.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            if (F_joint_after(i, j) == vi_after) {
                F_joint_after(i, j) = v_bd; // mapped to the boundary vertex
            } else {
                F_joint_after(i, j) = local_vid_after_to_before_map[F_joint_after(i, j)];
            }
        }
    }


    // bc
    Eigen::VectorXi b_UV;
    Eigen::VectorXd bc_UV;

    b_UV.resize(2 * 2, 1);
    bc_UV.resize(2 * 2, 1);
    int nVjoint = V_joint_before.rows();

    b_UV << vi_before, vj_before, vi_before + nVjoint, vj_before + nVjoint;
    bc_UV << 0, 1, 0, 0;

    // flatten
    flatten(V_joint_before, V_joint_after, F_joint_before, F_joint_after, b_UV, bc_UV, UV_joint);

    // modify UV_joint, F_after and v_id_map_joint
    UV_joint.conservativeResize(UV_joint.rows() + 1, UV_joint.cols());
    UV_joint.row(UV_joint.rows() - 1) = UV_joint.row(v_bd);

    F_after = F_joint_after;
    for (int i = 0; i < F_after.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            if (F_after(i, j) == v_bd) {
                F_after(i, j) = UV_joint.rows() - 1;
            }
        }
    }
    v_id_map_joint = v_id_map_before;
    v_id_map_joint.push_back(v_id_map_after[vi_after]);
}

void local_joint_flatten_case2_all_colinear(
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXd& V_before,
    const std::vector<int64_t>& v_id_map_before,
    Eigen::MatrixXi& F_after,
    const Eigen::MatrixXd& V_after,
    const std::vector<int64_t>& v_id_map_after,
    Eigen::MatrixXd& UV_joint,
    std::vector<int64_t>& v_id_map_joint)
{
    // get V_joint_before, F_joint_before, V_joint_after, F_joint_after
    // TODO: this could be easier if we get local mesh from a "good" order
    // this part is the same as case0
    int vi_after = 0;
    std::vector<int> local_vid_after_to_before_map(v_id_map_after.size(), -1);

    for (int i = 1; i < v_id_map_after.size(); i++) {
        auto it = std::find(v_id_map_before.begin(), v_id_map_before.end(), v_id_map_after[i]);
        if (it == v_id_map_before.end()) {
            std::runtime_error("There is vertex that is unituq in before!");
        }
        local_vid_after_to_before_map[i] = std::distance(v_id_map_before.begin(), it);
    }
    int vi_before = 0, vj_before = -1;
    for (int i = 0; i < v_id_map_before.size(); i++) {
        if (std::find(v_id_map_after.begin(), v_id_map_after.end(), v_id_map_before[i]) ==
            v_id_map_after.end()) {
            vj_before = i;
            break;
        }
    }
    if (vj_before == -1 || vj_before == vi_before) {
        throw std::runtime_error("Cannot find the joint vertex!");
    }

    // same as case 0
    Eigen::MatrixXd V_joint = V_before;
    V_joint.conservativeResize(V_joint.rows() + 1, V_joint.cols());
    // put vi_after to the end
    V_joint.row(V_joint.rows() - 1) = V_after.row(vi_after);
    Eigen::MatrixXi F_joint_before = F_before;
    Eigen::MatrixXi F_joint_after = F_after;
    // update F_joint after
    for (int i = 0; i < F_joint_after.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            if (F_joint_after(i, j) == vi_after) {
                F_joint_after(i, j) = V_joint.rows() - 1;
            } else {
                F_joint_after(i, j) = local_vid_after_to_before_map[F_joint_after(i, j)];
            }
        }
    }
    // modify F_after and v_id_map_joint
    F_after = F_joint_after;
    v_id_map_joint = v_id_map_before;
    v_id_map_joint.push_back(v_id_map_after[vi_after]);

    // get another vertices
    std::vector<std::vector<int>> bd_loops;
    igl::boundary_loop(F_joint_after, bd_loops);
    if (bd_loops.size() != 1) {
        for (auto& bd_loop : bd_loops) {
            for (auto& v : bd_loop) {
                std::cout << v << " ";
            }
            std::cout << std::endl;
        }
        throw std::runtime_error("bd loops size is not 1!");
    }
    auto bd_loop = bd_loops[0];
    auto it = std::find(bd_loop.begin(), bd_loop.end(), V_joint.rows() - 1);
    if (it == bd_loop.end()) {
        throw std::runtime_error("vi_after is not in bd loop!");
    }
    int idx = std::distance(bd_loop.begin(), it);

    Eigen::VectorXi b_UV;
    Eigen::VectorXd bc_UV;

    b_UV.resize(7, 1);
    bc_UV.resize(7, 1);
    bc_UV.setZero();

    b_UV(0) = vi_before + V_joint.rows();
    b_UV(1) = vj_before;
    bc_UV(1) = 1;
    b_UV(2) = vj_before + V_joint.rows();
    b_UV(3) = 2 * V_joint.rows() - 1; // vi_after in the end
    b_UV(4) = bd_loop[(idx + 1) % bd_loop.size()] + V_joint.rows();
    b_UV(5) = bd_loop[(idx + bd_loop.size() - 1) % bd_loop.size()] + V_joint.rows();
    b_UV(6) = vi_before;
    // flatten
    flatten(V_joint, V_joint, F_joint_before, F_joint_after, b_UV, bc_UV, UV_joint);

    Eigen::VectorXd E_before;
    quasi_conformal_energy(V_joint, F_joint_before, UV_joint, E_before);
    Eigen::VectorXd E_after;
    quasi_conformal_energy(V_joint, F_joint_after, UV_joint, E_after);

    std::cout << "E_before: \n" << E_before << std::endl;
    std::cout << "E_after: \n" << E_after << std::endl;
}


void local_joint_flatten_case2_3_colinear(
    const Eigen::MatrixXi& F_before,
    const Eigen::MatrixXd& V_before,
    const std::vector<int64_t>& v_id_map_before,
    Eigen::MatrixXi& F_after,
    const Eigen::MatrixXd& V_after,
    const std::vector<int64_t>& v_id_map_after,
    Eigen::MatrixXd& UV_joint,
    std::vector<int64_t>& v_id_map_joint)
{
    // get V_joint_before, F_joint_before, V_joint_after, F_joint_after
    // TODO: this could be easier if we get local mesh from a "good" order
    // this part is the same as case0
    int vi_after = 0;
    std::vector<int> local_vid_after_to_before_map(v_id_map_after.size(), -1);

    for (int i = 1; i < v_id_map_after.size(); i++) {
        auto it = std::find(v_id_map_before.begin(), v_id_map_before.end(), v_id_map_after[i]);
        if (it == v_id_map_before.end()) {
            std::runtime_error("There is vertex that is unituq in before!");
        }
        local_vid_after_to_before_map[i] = std::distance(v_id_map_before.begin(), it);
    }
    int vi_before = 0, vj_before = -1;
    for (int i = 0; i < v_id_map_before.size(); i++) {
        if (std::find(v_id_map_after.begin(), v_id_map_after.end(), v_id_map_before[i]) ==
            v_id_map_after.end()) {
            vj_before = i;
            break;
        }
    }
    if (vj_before == -1 || vj_before == vi_before) {
        throw std::runtime_error("Cannot find the joint vertex!");
    }

    // get two bds
    std::vector<std::vector<int>> bd_loops_before;
    igl::boundary_loop(F_before, bd_loops_before);
    if (bd_loops_before.size() != 1) {
        throw std::runtime_error("bd loops size is not 1!");
    }
    auto bd_loop_before = bd_loops_before[0];
    auto it_i = std::find(bd_loop_before.begin(), bd_loop_before.end(), vi_before);
    auto it_j = std::find(bd_loop_before.begin(), bd_loop_before.end(), vj_before);
    int it_i_idx = std::distance(bd_loop_before.begin(), it_i);
    int it_j_idx = std::distance(bd_loop_before.begin(), it_j);
    int bd_v_i = bd_loop_before[(it_i_idx + bd_loop_before.size() - 1) % bd_loop_before.size()];
    int bd_v_j = bd_loop_before[(it_j_idx + 1) % bd_loop_before.size()];
    if (bd_v_i == vj_before) {
        bd_v_i = bd_loop_before[(it_i_idx + 1) % bd_loop_before.size()];
        bd_v_j = bd_loop_before[(it_j_idx + bd_loop_before.size() - 1) % bd_loop_before.size()];
    }
    // decide which side to keep
    int v_to_keep = vi_before;
    int bd_v_to_keep = bd_v_j;
    bool find_all = false;
    for (int i = 0; i < F_before.rows(); i++) {
        int count = 0;
        for (int j = 0; j < 3; j++) {
            if (F_before(i, j) == vi_before || F_before(i, j) == vj_before ||
                F_before(i, j) == bd_v_i) {
                count++;
            }
        }
        if (count == 3) {
            find_all = true;
            break;
        }
    }
    if (find_all) {
        v_to_keep = vj_before;
        bd_v_to_keep = bd_v_i;
    }

    Eigen::MatrixXd V_joint_before = V_before;
    Eigen::MatrixXd V_joint_after = V_joint_before;

    V_joint_after.row(v_to_keep) = V_after.row(vi_after);
    Eigen::MatrixXi F_joint_before = F_before;
    Eigen::MatrixXi F_joint_after = F_after;
    // update F_joint after
    for (int i = 0; i < F_joint_after.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            if (F_joint_after(i, j) == vi_after) {
                F_joint_after(i, j) = v_to_keep; // mapped to the boundary vertex
            } else {
                F_joint_after(i, j) = local_vid_after_to_before_map[F_joint_after(i, j)];
            }
        }
    }

    // bc
    Eigen::VectorXi b_UV;
    Eigen::VectorXd bc_UV;

    b_UV.resize(2 * 2 + 1, 1);
    bc_UV.resize(2 * 2 + 1, 1);

    b_UV << vi_before, vj_before, vi_before + V_joint_before.rows(),
        bd_v_to_keep + V_joint_before.rows(), bd_v_to_keep;
    bc_UV << 0, 0, 0, 1, 0;

    // flatten
    flatten(V_joint_before, V_joint_after, F_joint_before, F_joint_after, b_UV, bc_UV, UV_joint);

    // modify UV_joint, F_after and v_id_map_joint
    UV_joint.conservativeResize(UV_joint.rows() + 1, UV_joint.cols());
    UV_joint.row(UV_joint.rows() - 1) = UV_joint.row(v_to_keep);
    std::cout << UV_joint << std::endl;
    F_after = F_joint_after;
    for (int i = 0; i < F_after.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            if (F_after(i, j) == v_to_keep) {
                F_after(i, j) = UV_joint.rows() - 1;
            }
        }
    }
    v_id_map_joint = v_id_map_before;
    v_id_map_joint.push_back(v_id_map_after[vi_after]);
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
    auto F_after_in = F_after;
    if (!is_bd_v0 && !is_bd_v1) {
        // std::cout << "case 0: interior edge" << std::endl;
        local_joint_flatten_case0(
            F_before,
            V_before,
            v_id_map_before,
            F_after,
            V_after,
            v_id_map_after,
            UV_joint,
            v_id_map_joint);
    } else if (is_bd_v0 && is_bd_v1) {
        std::cout << "case 2: boundary edge" << std::endl;
        local_joint_flatten_case2_3_colinear(
            F_before,
            V_before,
            v_id_map_before,
            F_after,
            V_after,
            v_id_map_after,
            UV_joint,
            v_id_map_joint);


    } else {
        std::cout << "case 1: edge connect a interior vertex and a boundary vertex" << std::endl;

        local_joint_flatten_case1(
            F_before,
            V_before,
            v_id_map_before,
            F_after,
            V_after,
            v_id_map_after,
            UV_joint,
            v_id_map_joint,
            is_bd_v0,
            is_bd_v1);
    }

    // check if all element in UV_joint is valid number
    bool check_nan = false;
    for (int i = 0; i < UV_joint.rows(); i++) {
        for (int j = 0; j < UV_joint.cols(); j++) {
            if (std::isnan(UV_joint(i, j))) {
                std::cout << "F_before:\n" << F_before << std::endl;
                std::cout << "V_before:\n" << V_before << std::endl;
                std::cout << "F_after:\n" << F_after_in << std::endl;
                std::cout << "V_after:\n" << V_after << std::endl;

                std::cout << "UV_joint:\n" << UV_joint << std::endl;
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
    Eigen::MatrixXd V_joint = V_before;
    V_joint.conservativeResize(V_joint.rows() + 1, V_joint.cols());
    V_joint.row(V_joint.rows() - 1) = V_after.row(0);

    Eigen::MatrixXi F_joint_before = F_before;
    Eigen::MatrixXi F_joint_after = F_before;
    // update F_joint after
    for (int i = 0; i < F_joint_after.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            if (F_joint_after(i, j) == 0) {
                F_joint_after(i, j) = V_joint.rows() - 1;
            }
        }
    }

    F_after = F_joint_after;
    // bc
    Eigen::VectorXi b_UV;
    Eigen::VectorXd bc_UV;

    b_UV.resize(3, 1);
    bc_UV.resize(3, 1);
    int nVjoint = V_before.rows();
    int vi_before = 1, vj_before = 2;

    b_UV << vi_before, vj_before, vi_before + nVjoint;
    bc_UV << 0, 1, 0;

    flatten(V_joint, V_joint, F_joint_before, F_joint_after, b_UV, bc_UV, UV_joint);
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

    // bc
    Eigen::VectorXi b_UV;
    Eigen::VectorXd bc_UV;

    b_UV.resize(3, 1);
    bc_UV.resize(3, 1);
    int nVjoint = V_before.rows();
    int vi_before = 0, vj_before = 1;

    b_UV << vi_before, vj_before, vi_before + nVjoint;
    bc_UV << 0, 1, 0;

    // flatten
    flatten(V_before, V_before, F_before, F_after_joint, b_UV, bc_UV, UV_joint);
}

} // namespace wmtk::operations::utils