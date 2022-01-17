#pragma once

#include <wmtk/ConcurrentTetMesh.h>


void wmtk::ConcurrentTetMesh::init(
    size_t n_vertices,
    const std::vector<std::array<size_t, 4>>& tets)
{
    wmtk::TetMesh::init(n_vertices, tets);
    m_vertex_mutex.grow_to_at_least(n_vertices);
}


int wmtk::ConcurrentTetMesh::release_vertex_mutex_in_stack(std::vector<size_t>& mutex_release_stack)
{
    int num_released = 0;
    for (int i = mutex_release_stack.size() - 1; i >= 0; i--) {
        unlock_vertex_mutex(mutex_release_stack[i]);
        mutex_release_stack.pop_back();
        num_released++;
    }
    return num_released;
}

bool wmtk::ConcurrentTetMesh::try_set_vertex_mutex_two_ring(
    Tuple& v,
    std::vector<size_t>& mutex_release_stack)
{
    return false;
}

bool wmtk::ConcurrentTetMesh::try_set_edge_mutex_two_ring(
    Tuple& e,
    std::vector<size_t>& mutex_release_stack)
{
    return false;
}