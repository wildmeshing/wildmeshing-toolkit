#include "compress_indices.hpp"
#include <wmtk/TriMesh.hpp>


std::pair<wmtk::MatrixXl, std::vector<wmtk::Tuple>> compress_indices(
    const wmtk::TriMesh& m,
    const wmtk::PrimitiveType& pt,
    const std::vector<wmtk::Tuple>& tups)
{
    wmtk::MatrixXl A;
    if (pt == wmtk::PrimitiveType::Vertex) {
        auto handle = m.get_attribute_handle<int64_t>("m_fv", wmtk::PrimitiveType::Triangle);
        auto acc = m.create_const_accessor<int64_t>(handle);
        A.resize(tups.size());
        for (int j = 0; j < A.size(); ++j) {
            A(j) = j;
        }
    }
    return {A, tups};
}
