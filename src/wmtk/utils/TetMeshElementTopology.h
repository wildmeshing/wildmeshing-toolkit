#pragma once
#include <array>


namespace wmtk::utils::tet_element_topology {

    namespace {
    /**
     * @brief local edges within a tet
     *
     */
    static constexpr std::array<std::array<int, 2>, 6> local_edges = {
        {{{0, 1}}, {{1, 2}}, {{0, 2}}, {{0, 3}}, {{1, 3}}, {{2, 3}}}};

    static constexpr std::array<int, 4> map_vertex2edge = {{0, 0, 1, 3}};
    static constexpr std::array<int, 4> map_vertex2oppo_face = {{3, 1, 2, 0}};
    static constexpr std::array<int, 6> map_edge2face = {{0, 0, 0, 1, 2, 1}};
    static constexpr std::array<std::array<int, 3>, 4> local_faces = {
        {{{0, 1, 2}}, {{0, 2, 3}}, {{0, 1, 3}}, {{1, 2, 3}}}}; // sorted local vids
    static constexpr std::array<std::array<int, 3>, 4> local_edges_in_a_face = {
        {{{0, 1, 2}}, {{2, 5, 3}}, {{3, 4, 0}}, {{5, 1, 4}}}};
    }
}
