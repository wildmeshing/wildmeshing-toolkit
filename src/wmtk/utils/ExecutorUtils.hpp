#pragma once

#include "TupleUtils.hpp"

namespace wmtk {

constexpr auto renewal_edges = [](const auto& m, auto op, const std::vector<TetMesh::Tuple>& newt) {
    std::vector<std::pair<std::string, TetMesh::Tuple>> op_tups;
    std::vector<TetMesh::Tuple> new_edges;
    for (const TetMesh::Tuple& ti : newt) {
        for (auto j = 0; j < 6; j++) {
            new_edges.push_back(m.tuple_from_edge(ti.tid(m), j));
        }
    };
    wmtk::unique_edge_tuples(m, new_edges);
    for (const auto& f : new_edges) {
        op_tups.emplace_back(op, f);
    }
    return op_tups;
};

constexpr auto renewal_faces = [](const auto& m, auto op, const auto& newtets) {
    std::vector<std::pair<std::string, wmtk::TetMesh::Tuple>> op_tups;

    auto new_faces = std::vector<wmtk::TetMesh::Tuple>();
    for (auto ti : newtets) {
        for (auto j = 0; j < 4; j++) new_faces.push_back(m.tuple_from_face(ti.tid(m), j));
    }
    wmtk::unique_face_tuples(m, new_faces);
    for (auto f : new_faces) op_tups.emplace_back(op, f);
    return op_tups;
};

constexpr auto renewal_simple = [](const auto& m, auto op, const auto& newts) {
    std::vector<std::pair<std::string, wmtk::TetMesh::Tuple>> op_tups;
    for (auto t : newts) {
        op_tups.emplace_back(op, t);
    }
    return op_tups;
};
} // namespace wmtk