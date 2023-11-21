#include "create_tag.hpp"
namespace wmtk::multimesh::utils {
std::set<long> create_tags(Mesh& m, const std::set<long>& critical_vids)
{
    internal::TupleTag tuple_tag(m, critical_vids);
    return tuple_tag.run();
}

} // namespace wmtk::multimesh::utils
