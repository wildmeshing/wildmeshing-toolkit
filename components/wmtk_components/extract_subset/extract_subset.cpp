#include "extract_subset.hpp"


namespace wmtk{
namespace components {
wmtk::TriMesh extract_subset(long dimension, const wmtk::TriMesh& m, std::vector<size_t> tag){
    switch (dimension){
        case 2: {
            // return internal::extrace_subset_2d();
            std::vector<Tuple> vertices = m.get_all(wmtk::PrimitiveType::Vertex);
            std::vector<Tuple> triangles = m.get_all(wmtk::PrimitiveType::Face);
            return internal::extract_subset_2d(vertices, triangles, tag);
        }
        case 3: {
            // to be implemented
        }
    }

}

}
}