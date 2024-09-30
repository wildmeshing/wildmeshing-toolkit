
#include "NavigatableSimplex.hpp"

#include <wmtk/Mesh.hpp>
namespace wmtk::simplex {

    NavigatableSimplex::NavigatableSimplex(const Mesh& m, const Simplex& s) {
    }

    NavigatableSimplex::operator Simplex () const {
        return {primitive_type(), tuple()};
    }
}
