#pragma once

#include <wmtk/TriMesh.hpp>

namespace wmtk {
namespace components {
namespace internal {

class IsotropicRemeshing
{
    TriMesh* m_mesh;
    double m_length_min = std::numeric_limits<double>::max();
    double m_length_max = std::numeric_limits<double>::lowest();

public:
    IsotropicRemeshing(TriMesh* mesh, const double length);

    void remeshing(const long iterations);

    void split_long_edges();
    void collapse_short_edges();
    void flip_edges_for_valence();
    void smooth_vertices();
};

} // namespace internal
} // namespace components
} // namespace wmtk