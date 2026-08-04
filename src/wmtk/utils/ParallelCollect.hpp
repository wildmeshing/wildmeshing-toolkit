#pragma once

#include <wmtk/ExecutionScheduler.hpp> // for wmtk::Op
#include <wmtk/threading/collector.hpp>
#include <wmtk/threading/parallel_for.hpp>

#include <utility>
#include <vector>

namespace wmtk {

// Build an operation list ("prepare") in parallel by reconstructing the canonical
// simplices over the cell range and appending per chunk (a single merge lock per chunk,
// not per element). This replaces the serial `for (auto& t : get_edges()/get_faces())
// collect.emplace_back(...)` loops. The resulting order differs from serial, but the
// executor's priority queue reorders it, so the processed set is identical.
//
// "Cell" is the top-dimensional element: a tet for TetMesh, a triangle for TriMesh. Both
// meshes expose cell_capacity() / tuple_from_cell() / EDGES_PER_CELL, so the edge version
// below is dimension-generic. `parallel_collect_face_ops` is tet-only (a TriMesh has no
// faces below its cells).
//
// `emit(mesh, simplex_tuple, local_out)` appends the desired op(s) for one simplex.

template <class Mesh, class Emit>
std::vector<std::pair<Op, typename Mesh::Tuple>>
parallel_collect_edge_ops(Mesh& m, int num_threads, Emit&& emit)
{
    using Tuple = typename Mesh::Tuple;
    constexpr size_t n_edges = Mesh::EDGES_PER_CELL;
    threading::collector<std::pair<Op, Tuple>> collect;

    threading::parallel_for(
        threading::range(0, m.cell_capacity()),
        [&](const threading::range& r) {
            std::vector<std::pair<Op, Tuple>> local;
            for (size_t i = r.begin(); i < r.end(); i++) {
                if (!m.tuple_from_cell(i).is_valid(m)) {
                    continue;
                }
                for (size_t j = 0; j < n_edges; j++) {
                    const Tuple e = m.tuple_from_edge(i, j);
                    if (e.eid(m) == n_edges * i + j) {
                        emit(m, e, local); // canonical edge only
                    }
                }
            }
            if (local.empty()) {
                return;
            }
            collect.append(local);
        },
        num_threads);

    return collect.data();
}

template <class Mesh, class Emit>
std::vector<std::pair<Op, typename Mesh::Tuple>>
parallel_collect_face_ops(Mesh& m, int num_threads, Emit&& emit)
{
    using Tuple = typename Mesh::Tuple;
    constexpr size_t n_faces = Mesh::FACES_PER_CELL;
    static_assert(
        n_faces > 0,
        "parallel_collect_face_ops requires a mesh with faces below its cells");
    threading::collector<std::pair<Op, Tuple>> collect;

    threading::parallel_for(
        threading::range(0, m.cell_capacity()),
        [&](const threading::range& r) {
            std::vector<std::pair<Op, Tuple>> local;
            for (size_t i = r.begin(); i < r.end(); i++) {
                if (!m.tuple_from_cell(i).is_valid(m)) {
                    continue;
                }
                for (size_t j = 0; j < n_faces; j++) {
                    const Tuple f = m.tuple_from_face(i, j);
                    if (f.fid(m) == n_faces * i + j) {
                        emit(m, f, local); // canonical face only
                    }
                }
            }
            if (local.empty()) {
                return;
            }
            collect.append(local);
        },
        num_threads);

    return collect.data();
}

} // namespace wmtk
