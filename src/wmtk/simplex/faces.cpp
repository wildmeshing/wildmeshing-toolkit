#include "faces.hpp"
#include "boundary.hpp"


namespace wmtk::simplex {
SimplexCollection faces(const Mesh& mesh, const Simplex& simplex, const bool sort_and_clean)
{
    SimplexCollection collection(mesh);

    const Mesh& m = mesh;
    const Tuple& t = simplex.tuple();


    for (const Simplex& s : boundary(mesh, simplex, false)) {
        collection.add(s);
        collection.add(faces(m, s, false));
    }


    if (sort_and_clean) {
        collection.sort_and_clean();
    }

    return collection;
}

} // namespace wmtk::simplex
