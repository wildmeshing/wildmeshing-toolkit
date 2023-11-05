#include "extract_subset.hpp"


namespace wmtk{
namespace components {
// wmtk::TriMesh extract_subset(const wmtk::TriMesh& m, std::vector<size_t> tag, long dimension){
//     switch (dimension){
//         case 2: {
//             // return internal::extrace_subset_2d();
//             std::vector<Tuple> vertices = m.get_all(wmtk::PrimitiveType::Vertex);
//             std::vector<Tuple> triangles = m.get_all(wmtk::PrimitiveType::Face);
//             return internal::extract_subset_2d(vertices, triangles, tag);
//         }
//         case 3: {
//             // to be implemented
//             throw std::runtime_error("not implemented");
//         }
//     }
// }

wmtk::TriMesh extract_subset(wmtk::TriMesh m, wmtk::MeshAttributeHandle<long> tag_handle, long dimension){
    switch (dimension){
        case 2: {
            return internal::extract_subset_2d(m, tag_handle);
        }
        case 3: {
            // to be implemented
            throw std::runtime_error("not implemented");
        }
    }
}

}
}