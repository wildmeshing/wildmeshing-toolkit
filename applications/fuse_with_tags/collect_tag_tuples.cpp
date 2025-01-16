#include "collect_tag_tuples.hpp"
#include <wmtk/Mesh.hpp>

std::map<int64_t, std::vector<wmtk::Tuple>> compress_indices(
    const wmtk::attribute::MeshAttributeHandle& h)
{
    auto acc = h.mesh().create_const_accessor<int64_t>(h);
    std::map<int64_t, std::vector<wmtk::Tuple>> ret;
    for (const wmtk::Tuple& t : h.mesh().get_all(h.primitive_type())) {
        ret[acc.const_scalar_attribute(t)].emplace_back(t);
    }

    return ret;
}
