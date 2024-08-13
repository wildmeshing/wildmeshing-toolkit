#pragma once

// DO NOT MODIFY, autogenerated from the /scripts directory

#include <cstdint>
namespace wmtk::autogen::tri_mesh {
// lvids for a particular leid
extern const int64_t auto_2d_edges[3][2];

// vertex switch's (tuple_index -> [lvid,leid])
extern const int64_t auto_2d_table_vertex[9][2];

// edge switch's (tuple_index -> [lvid,leid])
extern const int64_t auto_2d_table_edge[9][2];

// (tuple_index) -> is_ccw
extern const int64_t auto_2d_table_ccw[9];

// lvid -> a ccw [lvid,leid]
extern const int64_t auto_2d_table_complete_vertex[3][2];

// leid -> a ccw [lvid,leid]
extern const int64_t auto_2d_table_complete_edge[3][2];

// Valid tuple local indices
extern const int8_t auto_valid_tuples[6][2];

// For each valid tuple encodes the raw tuple index
extern const int8_t auto_valid_tuple_indices[6];

// Index of each tuple according to valid tuple indexing
extern const int8_t auto_all_to_valid_tuple_indices[9];

// Valid tuple indices
extern const int8_t auto_valid_switch_table[6][2];

// Tuple group product using valid tuple indices
extern const int8_t auto_valid_switch_product_table[6][6];

// Tuple group product inverse using valid tuple indices
extern const int8_t auto_valid_switch_inverse_table[6];

// Which tuples are associated with switching. Last two entries are the identity action and opp
// action
extern const int8_t auto_valid_tuple_switch_indices[4];


} // namespace wmtk::autogen::tri_mesh
