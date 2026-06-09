#include "TriWildMesh.h"

#include <igl/Timer.h>
#include <wmtk/TriMesh.h>
#include <wmtk/ExecutionScheduler.hpp>
#include <wmtk/utils/ExecutorUtils.hpp>
#include <wmtk/utils/Logger.hpp>
#include "wmtk/utils/TupleUtils.hpp"

#include <cassert>

namespace wmtk::components::triwild {

size_t TriWildMesh::swap_all_edges()
{
    log_and_throw_error("edge swapping is not implemented yet");
}

bool TriWildMesh::swap_edge_before(const Tuple& t)
{
    log_and_throw_error("edge swapping is not implemented yet");
}

bool TriWildMesh::swap_edge_after(const Tuple& t)
{
    log_and_throw_error("edge swapping is not implemented yet");
}


} // namespace wmtk::components::triwild