#include <array>
#include <catch2/catch_test_macros.hpp>
#include <set>
#include <wmtk/TetMesh.hpp>
#include <wmtk/TriMesh.hpp>
#include <wmtk/simplex/Simplex.hpp>
#include <wmtk/simplex/SimplexCollection.hpp>
#include <wmtk/simplex/boundary.hpp>
#include <wmtk/simplex/closed_star.hpp>
#include <wmtk/simplex/closed_star_iterable.hpp>
#include <wmtk/simplex/cofaces_single_dimension.hpp>
#include <wmtk/simplex/faces.hpp>
#include <wmtk/simplex/top_dimension_cofaces.hpp>
#include <wmtk/simplex/top_dimension_cofaces_iterable.hpp>
#include <wmtk/simplex/utils/SimplexComparisons.hpp>
#include <wmtk/simplex/utils/tuple_vector_to_homogeneous_simplex_vector.hpp>
#include <wmtk/utils/primitive_range.hpp>
#include "tools/DEBUG_EdgeMesh.hpp"
#include "tools/DEBUG_TetMesh.hpp"
#include "tools/DEBUG_TriMesh.hpp"
#include "tools/EdgeMesh_examples.hpp"
#include "tools/TetMesh_examples.hpp"
#include "tools/TriMesh_examples.hpp"
#include "tools/all_valid_local_tuples.hpp"

using namespace wmtk;
using namespace simplex;

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
constexpr PrimitiveType PF = PrimitiveType::Face;
constexpr PrimitiveType PT = PrimitiveType::Tetrahedron;

