

//
#include "CDT.hpp"
#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/mesh_utils.hpp>
#include <wmtk/utils/orient.hpp>
#include "cdt_lib.hpp"
#include "get_vf.hpp"

namespace wmtk::components::internal {

std::shared_ptr<wmtk::TetMesh> CDT_internal(
    const wmtk::TriMesh& m,
    std::vector<std::array<bool, 4>>& local_f_on_input,
    bool inner_only,
    bool rational_output)
{
    auto [vdat, fdat] = get_vf(m);
    std::vector<double> V_tmp;
    uint32_t npts, ntri;
    std::tie(V_tmp, npts) = vdat;
    std::vector<uint32_t> F;

    std::tie(F, ntri) = fdat;

    std::vector<std::array<int64_t, 4>> T_final;
    std::vector<std::array<std::string, 3>> V_final_str;

    cdt_lib::cdt_to_string(
        V_tmp,
        npts,
        F,
        ntri,
        local_f_on_input,
        T_final,
        V_final_str,
        inner_only);


    MatrixX<int64_t> T;

    T.resize(T_final.size(), 4);

    for (int64_t i = 0; i < T_final.size(); ++i) {
        T(i, 0) = T_final[i][0];
        T(i, 1) = T_final[i][1];
        T(i, 2) = T_final[i][2];
        T(i, 3) = T_final[i][3];
    }

    std::shared_ptr<wmtk::TetMesh> tm = std::make_shared<wmtk::TetMesh>();
    tm->initialize(T);


    MatrixX<Rational> V;
    V.resize(V_final_str.size(), 3);

    for (int64_t i = 0; i < V_final_str.size(); ++i) {
        V(i, 0).init_from_binary(V_final_str[i][0]);
        V(i, 1).init_from_binary(V_final_str[i][1]);
        V(i, 2).init_from_binary(V_final_str[i][2]);
    }

    if (rational_output) {
        // check inversion
        for (int64_t i = 0; i < T_final.size(); ++i) {
            if (wmtk::utils::wmtk_orient3d(
                    V.row(T(i, 0)),
                    V.row(T(i, 1)),
                    V.row(T(i, 2)),
                    V.row(T(i, 3))) < 0) {
                auto tmp = T(i, 0);
                T(i, 0) = T(i, 1);
                T(i, 1) = tmp;

                wmtk::logger().info("inverted tet rational, swap 0 1");
            }
        }

        mesh_utils::set_matrix_attribute(V, "vertices", PrimitiveType::Vertex, *tm);
    } else {
        MatrixX<double> V_double;
        V_double.resize(V_final_str.size(), 3);

        V_double = V.cast<double>();

        // check inversion
        for (int64_t i = 0; i < T_final.size(); ++i) {
            if (wmtk::utils::wmtk_orient3d(
                    V_double.row(T(i, 0)),
                    V_double.row(T(i, 1)),
                    V_double.row(T(i, 2)),
                    V_double.row(T(i, 3))) < 0) {
                auto tmp = T(i, 0);
                T(i, 0) = T(i, 1);
                T(i, 1) = tmp;

                wmtk::logger().info("inverted tet double, swap 0 1");
            }
        }

        mesh_utils::set_matrix_attribute(V_double, "vertices", PrimitiveType::Vertex, *tm);
    }


    return tm;
}

} // namespace wmtk::components::internal
