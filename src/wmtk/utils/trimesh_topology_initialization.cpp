#include "trimesh_topology_initialization.h"
#include <algorithm>
#include <vector>
#include <wmtk/autogen/tri_mesh/autogenerated_tables.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk {


std::tuple<RowVectors3l, RowVectors3l, VectorXl, VectorXl> trimesh_topology_initialization(
    Eigen::Ref<const RowVectors3l> F)
{
    RowVectors3l FE, FF;
    VectorXl VF, EF;

    // Make sure there are 3 columns
    assert(F.cols() == 3);

    char iv0 = 0;
    char iv1 = 1;
    char it = 2;
    char ii = 3;

    std::vector<std::vector<int64_t>> TTT;

    int64_t vertex_count = F.maxCoeff() + 1;

    // Build a table for finding Faces and populate the corresponding
    // topology relations
    {
        TTT.resize(F.rows() * 3);
        for (int t = 0; t < F.rows(); ++t) {
            for (int i = 0; i < 3; ++i) {
                // v1 v2 v3 f ei
                const auto& [f0, f1] = wmtk::autogen::tri_mesh::auto_2d_edges[i];
                int64_t x = F(t, f0);
                int64_t y = F(t, f1);
                if (x > y) std::swap(x, y);

                std::vector<int64_t> r(4);
                r[iv0] = x;
                r[iv1] = y;
                r[it] = t;
                r[ii] = i;
                TTT[t * 3 + i] = r;
            }
        }
        std::sort(TTT.begin(), TTT.end());

        // VF
        VF = VectorXl::Constant(vertex_count, 1, -1);
        for (int i = 0; i < F.rows(); ++i) {
            for (int j = 0; j < 3; ++j) {
                VF[F(i, j)] = i;
            }
        }

        // Compute FE, FF, EF
        FE.resize(F.rows(), 3);
        FF.resize(F.rows(), 3);
        std::vector<int64_t> EF_temp;

        // iterate over TTT to find faces
        // for every entry check if the next is the same, and update the connectivity accordingly

        for (int i = 0; i < TTT.size(); ++i) {
            if ((i == (TTT.size() - 1)) || (TTT[i][0] != TTT[i + 1][0]) ||
                (TTT[i][1] != TTT[i + 1][1])) {
                // If the next tuple is empty, then this is a boundary edge
                EF_temp.push_back(TTT[i][it]);

                FF(TTT[i][it], TTT[i][ii]) = -1;
                FE(TTT[i][it], TTT[i][ii]) = EF_temp.size() - 1;
            } else {
                // this is an internal edge, update both sides
                EF_temp.push_back(TTT[i][it]);

                FF(TTT[i][it], TTT[i][ii]) = TTT[i + 1][it];
                FE(TTT[i][it], TTT[i][ii]) = EF_temp.size() - 1;

                FF(TTT[i + 1][it], TTT[i + 1][ii]) = TTT[i][it];
                FE(TTT[i + 1][it], TTT[i + 1][ii]) = EF_temp.size() - 1;

                ++i; // skip the other entry
            }
        }

        // copy EF
        EF.resize(EF_temp.size());
        for (int64_t i = 0; i < EF_temp.size(); ++i) EF(i) = EF_temp[i];
    }

    return {FE, FF, VF, EF};
}

} // namespace wmtk