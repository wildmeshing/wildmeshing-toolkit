#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json topological_offset_spec = R"(
[
  {
    "pointer": "/",
    "type": "object",
    "required": [
      "application",
      "input"
    ],
    "optional": [
      "output",
      "offset_tags",
      "offset_tag_val",
      "target_distance",
      "relative_ball_threshold",
      "edge_search_termination_len",
      "sorted_marching",
      "check_manifoldness",
      "save_vtu",
      "DEBUG_output"
    ]
  },
  {
    "pointer": "/application",
    "type": "string",
    "options": [
      "topological_offset"
    ],
    "doc": "Application name must be topological_offset."
  },
  {
    "pointer": "/input",
    "type": "string",
    "doc": "Tetrahedral input mesh."
  },
  {
    "pointer": "/output",
    "type": "string",
    "default": "out",
    "doc": "Output file name (without extension)."
  },
  {
    "pointer": "/offset_tags",
    "type": "list",
    "default": [
      [
        0,
        1
      ]
    ],
    "min": 1,
    "doc": "List of [tag num, tag val] pairs. If one given, that region is offset. If multiple, the intersection of all is offset."
  },
  {
    "pointer": "/offset_tags/*",
    "type": "list",
    "min": 2,
    "max": 2
  },
  {
    "pointer": "/offset_tags/*/*",
    "type": "int"
  },
  {
    "pointer": "/offset_tag_val",
    "type": "list",
    "default": [
      [
        0,
        0
      ]
    ],
    "min": 1,
    "doc": "[Tag num, tag val] pairs to set for offset result. Missing tag nums inherit from parent tris/tets."
  },
  {
    "pointer": "/offset_tag_val/*",
    "type": "list",
    "min": 2,
    "max": 2
  },
  {
    "pointer": "/offset_tag_val/*/*",
    "type": "int"
  },
  {
    "pointer": "/target_distance",
    "type": "float",
    "default": -1.0,
    "doc": "Target distance for offset. If negative, the offset is created by splitting adjacent edges at their midpoints."
  },
  {
    "pointer": "/relative_ball_threshold",
    "type": "float",
    "default": 0.1,
    "doc": "Radius relative to target_distance to stop circle (2d) or sphere (3d) splitting in conservative distance approximation."
  },
  {
    "pointer": "/edge_search_termination_len",
    "type": "float",
    "default": 1e-3,
    "doc": "Length below which binary search will be terminated for function-guided edge splitting"
  },
  {
    "pointer": "/sorted_marching",
    "type": "bool",
    "default": false,
    "doc": "Execute marching tets in decreasing order of edge length. Increases run time, may increase output mesh quality."
  },
  {
    "pointer": "/check_manifoldness",
    "type": "bool",
    "default": true,
    "doc": "After performing offset, check if offset region is manifold"
  },
  {
    "pointer": "/save_vtu",
    "type": "bool",
    "default": false,
    "doc": "Save .vtu of output mesh"
  },
  {
    "pointer": "/DEBUG_output",
    "type": "bool",
    "default": false,
    "doc": "Write the tet mesh as out_{}.vtu after every operation."
  }
]
)"_json;

}