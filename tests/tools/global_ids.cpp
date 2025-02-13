
#include "global_ids.hpp"
#include <spdlog/spdlog.h>
#include <wmtk/Mesh.hpp>
#include "TestTools.hpp"
namespace wmtk::tests::tools {
std::array<int64_t, 4> TestTools::global_ids(const Mesh& m, const Tuple& a)
{
    std::array<int64_t, 4> ret = {{-1, -1, -1, -1}};
    for (PrimitiveType pt = PrimitiveType::Vertex; pt < m.top_simplex_type(); pt = pt + 1) {
        spdlog::info("Assigning to {} value {}", int(pt), m.id(a, pt));
        ret[int(pt)] = m.id(a, pt);
    }
    ret[m.top_cell_dimension()] = a.global_cid();
    return ret;
}
std::array<int64_t, 4> global_ids(const Mesh& m, const Tuple& a)
{
    return TestTools::global_ids(m, a);
}
} // namespace wmtk::tests::tools
