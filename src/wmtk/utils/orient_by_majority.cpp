#include "orient_by_majority.hpp"

#include <cassert>
#include <numeric>
#include <utility>

namespace wmtk::utils {

OrientationRepair orient_by_majority(
    size_t n,
    const std::vector<OrientationLink>& links,
    const std::vector<double>& weight)
{
    assert(weight.size() == n);

    // Union-find with parity: parity[x] is 1 iff x and parent[x] must end up with opposite
    // orientations for every link inside their component to be consistent.
    std::vector<uint32_t> parent(n);
    std::iota(parent.begin(), parent.end(), uint32_t(0));
    std::vector<uint8_t> parity(n, 0);
    std::vector<uint32_t> size(n, 1);
    std::vector<uint8_t> contradicted(n, 0); // per root

    // The root of x, and x's parity against it; compresses the path.
    const auto find = [&](uint32_t x) {
        uint32_t r = x;
        uint8_t p = 0;
        while (parent[r] != r) {
            p ^= parity[r];
            r = parent[r];
        }
        uint32_t y = x;
        uint8_t py = p;
        while (parent[y] != y) {
            const uint32_t next = parent[y];
            const uint8_t p_next = py ^ parity[y];
            parent[y] = r;
            parity[y] = py;
            y = next;
            py = p_next;
        }
        return std::make_pair(r, p);
    };

    for (const OrientationLink& l : links) {
        assert(l.a < n && l.b < n);
        auto [ra, pa] = find(l.a);
        auto [rb, pb] = find(l.b);
        const uint8_t needed = l.consistent ? 0 : 1;
        if (ra == rb) {
            if ((pa ^ pb) != needed) contradicted[ra] = 1;
            continue;
        }
        if (size[ra] < size[rb]) {
            std::swap(ra, rb);
            std::swap(pa, pb);
        }
        parent[rb] = ra;
        parity[rb] = pa ^ pb ^ needed;
        size[ra] += size[rb];
        contradicted[ra] |= contradicted[rb];
    }

    // Per component, the weight on each side of its root.
    std::vector<double> with_root(n, 0), against_root(n, 0);
    std::vector<uint32_t> root(n);
    std::vector<uint8_t> par(n);
    for (uint32_t i = 0; i < n; ++i) {
        const auto [r, p] = find(i);
        root[i] = r;
        par[i] = p;
        (p ? against_root : with_root)[r] += weight[i];
    }

    OrientationRepair res;
    res.turn.assign(n, false);
    // Per root: whether its side is the one to reverse, decided at its component's lowest
    // element (the first one the loop meets): -1 undecided, else 0 / 1.
    std::vector<int8_t> turn_root_side(n, -1);
    for (uint32_t i = 0; i < n; ++i) {
        const uint32_t r = root[i];
        if (turn_root_side[r] < 0) {
            if (contradicted[r]) {
                ++res.n_non_orientable;
                turn_root_side[r] = 0;
            } else if (against_root[r] != with_root[r]) {
                // Reverse the lighter side.
                turn_root_side[r] = against_root[r] > with_root[r] ? 1 : 0;
            } else {
                // A tie: keep the side this element is on.
                turn_root_side[r] = par[i] ? 1 : 0;
            }
        }
        if (contradicted[r]) continue;
        res.turn[i] = (par[i] != 0) != (turn_root_side[r] != 0);
        if (res.turn[i]) ++res.n_turned;
    }
    return res;
}

} // namespace wmtk::utils
