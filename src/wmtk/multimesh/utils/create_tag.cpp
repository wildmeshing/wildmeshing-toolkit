#include "create_tag.hpp"
namespace wmtk::multimesh::utils {
std::set<int64_t> create_tags(Mesh& m, const std::set<int64_t>& critical_vids)
{
    internal::TupleTag tuple_tag(m, critical_vids);
    return tuple_tag.run();
}

} // namespace wmtk::multimesh::utils
