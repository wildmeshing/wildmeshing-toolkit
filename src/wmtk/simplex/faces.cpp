#include "faces.hpp"
#include "boundary.hpp"

#include <wmtk/utils/primitive_range.hpp>

#include "faces_single_dimension.hpp"


namespace wmtk::simplex {
SimplexCollection faces(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(mesh);

    const auto primitive_range = wmtk::utils::primitive_below(simplex.primitive_type());
    for (size_t i = 1; i < primitive_range.size(); ++i) {
        collection.add(faces_single_dimension(mesh, simplex, primitive_range[i]));
    }


    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

} // namespace wmtk::simplex
