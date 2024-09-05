#include "global_index_max_subdart_size.hpp"
#include <cassert>
#include <wmtk/Mesh.hpp>
#include "TestTools.hpp"
namespace wmtk::tests::tools {
int8_t TestTools::global_index_max_subdart_size(const Mesh& m, const Tuple& a, const Tuple& b)
{
    int8_t count = 0;
    for (PrimitiveType pt = PrimitiveType::Vertex; pt <= m.top_simplex_type(); pt = pt + 1) {
        if (pt == m.top_simplex_type()) {
            count += 1;
        } else if (m.id(a, pt) != m.id(b, pt)) {
            break;

        } else {
            count += 1;
        }
    }
    return count;
}
int8_t global_index_max_subdart_size(const Mesh& m, const Tuple& a, const Tuple& b)
{
    return TestTools::global_index_max_subdart_size(m, a, b);
}

int8_t global_index_max_subdart_size(
    const std::array<int64_t, 4>& a,
    const std::array<int64_t, 4>& b)
{
    for (size_t j = 0; j < 4; ++j) {
        // as long as a == b keep going, otherwise return (also some extra logic to add an assert)
        if (a[j] == b[j]) {
            continue;
        } else if (a[j] == -1 || b[j] == -1) {
            assert(
                a[j] == -1 &&
                b[j] == -1); // if one is -1 then both should be if htey're the same dim dart
        }
        return j;
    }
    return 4;
}
} // namespace wmtk::tests::tools
