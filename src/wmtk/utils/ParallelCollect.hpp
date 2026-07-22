#pragma once

#include <wmtk/ExecutionScheduler.hpp> // for wmtk::Op
#include <wmtk/threading/parallel_for.hpp>

#include <mutex>
#include <utility>
#include <vector>

namespace wmtk {

// Build an operation list ("prepare") in parallel by reconstructing the canonical
// simplices over the tet range and appending per chunk (a single merge lock per chunk,
// not per element). This replaces the serial `for (auto& t : get_edges()/get_faces())
// collect.emplace_back(...)` loops. The resulting order differs from serial, but the
// executor's priority queue reorders it, so the processed set is identical.
//
// `emit(mesh, simplex_tuple, local_out)` appends the desired op(s) for one simplex.

template <class Mesh, class Emit>
std::vector<std::pair<Op, typename Mesh::Tuple>>
parallel_collect_edge_ops(Mesh& m, int num_threads, Emit&& emit)
{
    using Tuple = typename Mesh::Tuple;
    std::vector<std::pair<Op, Tuple>> out;
    std::mutex merge_mutex;

    threading::parallel_for(
        threading::range(0, m.tet_capacity()),
        [&](const threading::range& r) {
            std::vector<std::pair<Op, Tuple>> local;
            for (size_t i = r.begin(); i < r.end(); i++) {
                if (!m.tuple_from_tet(i).is_valid(m)) continue;
                for (int j = 0; j < 6; j++) {
                    const Tuple e = m.tuple_from_edge(i, j);
                    if (e.eid(m) == 6 * i + j) emit(m, e, local); // canonical edge only
                }
            }
            if (local.empty()) return;
            std::lock_guard<std::mutex> lk(merge_mutex);
            out.insert(
                out.end(),
                std::make_move_iterator(local.begin()),
                std::make_move_iterator(local.end()));
        },
        num_threads);

    return out;
}

template <class Mesh, class Emit>
std::vector<std::pair<Op, typename Mesh::Tuple>>
parallel_collect_face_ops(Mesh& m, int num_threads, Emit&& emit)
{
    using Tuple = typename Mesh::Tuple;
    std::vector<std::pair<Op, Tuple>> out;
    std::mutex merge_mutex;

    threading::parallel_for(
        threading::range(0, m.tet_capacity()),
        [&](const threading::range& r) {
            std::vector<std::pair<Op, Tuple>> local;
            for (size_t i = r.begin(); i < r.end(); i++) {
                if (!m.tuple_from_tet(i).is_valid(m)) continue;
                for (int j = 0; j < 4; j++) {
                    const Tuple f = m.tuple_from_face(i, j);
                    if (f.fid(m) == 4 * i + j) emit(m, f, local); // canonical face only
                }
            }
            if (local.empty()) return;
            std::lock_guard<std::mutex> lk(merge_mutex);
            out.insert(
                out.end(),
                std::make_move_iterator(local.begin()),
                std::make_move_iterator(local.end()));
        },
        num_threads);

    return out;
}

} // namespace wmtk
