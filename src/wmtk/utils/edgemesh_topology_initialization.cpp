#include "edgemesh_topology_initialization.h"
#include <algorithm>
#include <vector>
#include <wmtk/utils/Logger.hpp>

namespace wmtk {

std::tuple<RowVectors2l, VectorXl> edgemesh_topology_initialization(
    Eigen::Ref<const RowVectors2l> E)
{
    RowVectors2l EE;
    VectorXl VE;

    const int64_t vertex_count = E.maxCoeff() + 1;

    // store the complete vertex-edge connnectivity
    std::vector<std::vector<int64_t>> complete_VE(vertex_count);

    // compute VE
    VE.resize(vertex_count, 1);
    for (int64_t i = 0; i < E.rows(); ++i) {
        for (int64_t j = 0; j < E.cols(); ++j) {
            VE[E(i, j)] = i;
            complete_VE[E(i, j)].push_back(i);
        }
    }

    EE.resize(E.rows(), 2);
    //  compute EE & connectivity check
    for (int64_t i = 0; i < complete_VE.size(); ++i) {
        assert(complete_VE[i].size() > 0 || complete_VE[i].size() < 3);
        if (complete_VE[i].size() == 1) {
            // boundary vertex
            if (E(complete_VE[i][0], 0) == i) {
                EE(complete_VE[i][0], 0) = -1;
            } else {
                EE(complete_VE[i][0], 1) = -1;
            }
        } else {
            // non-boundary vertex
            for (int64_t k = 0; k < 2; ++k) {
                if (E(complete_VE[i][k], 0) == i) {
                    EE(complete_VE[i][k], 0) = complete_VE[i][1 - k];
                }
                if (E(complete_VE[i][k], 1) == i) {
                    EE(complete_VE[i][k], 1) = complete_VE[i][1 - k];
                }
            }
        }
    }
    return {EE, VE};
}
} // namespace wmtk