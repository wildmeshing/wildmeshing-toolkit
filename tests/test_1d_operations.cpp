#include <catch2/catch_test_macros.hpp>

#include <numeric>
#include <wmtk/Accessor.hpp>
#include <wmtk/EdgeMeshOperationExecutor.hpp>
#include <wmtk/operations/OperationFactory.hpp>
#include <wmtk/utils/Logger.hpp>
#include "tools/DEBUG_EdgeMesh.hpp"
#include "tools/EdgeMesh_examples.hpp"

using namespace wmtk;
using namespace wmtk::tests;

using EM = EdgeMesh;
using MapResult = typename Eigen::Matrix<long, Eigen::Dynamic, 1>::MapType;
using TMOE = decltype(std::declval<DEBUG_EdgeMesh>().get_tmoe(
    wmtk::Tuple(),
    std::declval<Accessor<long>&>()));

constexpr PrimitiveType PV = PrimitiveType::Vertex;
constexpr PrimitiveType PE = PrimitiveType::Edge;
