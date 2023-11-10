#include "extract_subset.hpp"

namespace wmtk {
namespace components {

wmtk::TriMesh
extract_subset(wmtk::TriMesh m, wmtk::MeshAttributeHandle<long> tag_handle, long dimension)
{
    switch (dimension) {
    case 2: {
        return internal::extract_subset_2d(m, tag_handle);
    }
    case 3: {
        // to be implemented
        throw std::runtime_error("not implemented");
    }
    default: {
        // to be implemented
        throw std::runtime_error("not implemented");
    }
    }
}

} // namespace components
} // namespace wmtk