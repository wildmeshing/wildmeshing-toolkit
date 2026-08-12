#include "Recovery.hpp"

#include <algorithm>

namespace wmtk::components::prismatic_mesh {

PrismRecoveryPartition recover_prism_partition(
    const std::array<size_t, 6>& p,
    const std::array<bool, 3>& marked)
{
    PrismRecoveryPartition result;
    const size_t count = std::count(marked.begin(), marked.end(), true);
    if (count == 0) {
        result.prisms.push_back(p);
        return result;
    }

    if (count == 1) {
        const size_t i = static_cast<size_t>(std::find(marked.begin(), marked.end(), true) - marked.begin());
        // The pyramid occupies the part opposite the marked top vertex and the
        // tetrahedron fills the remaining corner. Their common triangular face
        // is internal, so the union has exactly the original prism boundary.
        if (i == 0) {
            result.pyramids.push_back({{p[1], p[4], p[5], p[2], p[3]}});
        } else if (i == 1) {
            result.pyramids.push_back({{p[2], p[5], p[3], p[0], p[4]}});
        } else {
            result.pyramids.push_back({{p[0], p[3], p[4], p[1], p[5]}});
        }
        result.tets.push_back({{p[0], p[1], p[2], p[3 + i]}});
        return result;
    }

    // A fixed body-diagonal partition is conforming across neighboring prism
    // candidates because it depends only on the established prism ordering.
    result.tets.push_back({{p[0], p[1], p[2], p[3]}});
    result.tets.push_back({{p[1], p[2], p[3], p[4]}});
    result.tets.push_back({{p[2], p[3], p[4], p[5]}});
    return result;
}

} // namespace wmtk::components::prismatic_mesh
