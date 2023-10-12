#include "extract_subset.hpp"

namespace wmtk{
namespace components {
wmtk::TriMesh extract_subset(long dimension, const wmtk::TriMesh& m, std::vector<size_t> tag){
    switch (dimension){
        case 2: {
            // return internal::extrace_subset_2d();
        }
        case 3: {
            // to be implemented
        }
    }

}

}
}