#include "IsotropicRemeshing.hpp"

namespace wmtk {
namespace components {
namespace internal {

IsotropicRemeshing::IsotropicRemeshing(TriMesh* mesh, const double length)
    : m_mesh{mesh}
    , m_length_min{(4. / 5.) * length}
    , m_length_max{(4. / 3.) * length}
{}

void IsotropicRemeshing::remeshing(const long iterations)
{
    for (long i = 0; i < iterations; ++i) {
        split_long_edges();
        collapse_short_edges();
        flip_edges_for_valence();
        smooth_vertices();
    }
}

void IsotropicRemeshing::split_long_edges()
{
    throw "implementation missing";
}

void IsotropicRemeshing::collapse_short_edges()
{
    throw "implementation missing";
}

void IsotropicRemeshing::flip_edges_for_valence()
{
    throw "implementation missing";
}

void IsotropicRemeshing::smooth_vertices()
{
    throw "implementation missing";
}


} // namespace internal
} // namespace components
} // namespace wmtk