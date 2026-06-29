#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json topological_offset_spec = R"(
[
  {
    "pointer": "/",
    "type": "object",
    "required": ["application", "input"],
    "optional": [
      "output",
      "input_dir",
      "offset_selection",
      "offset_output_tags",
      "protected_tags",
      "respect_all_topologies",
      "overwrite_tags",
      "offset_in",
      "offset_out",
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
    "options": ["topological_offset"],
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
    "pointer": "/input_dir",
    "type": "string",
    "default": "",
    "doc": "Directory where the input files are located. This is injected by the application and should not be set by the user."
  },
  {
    "pointer": "/offset_selection",
    "type": "string",
    "default": "!_",
    "doc": "Boolean expression for input simplicial complex to offset. If a single tag (ie 'tag_0') is given, single body mode is used."
  },
  {
    "pointer": "/offset_output_tags",
    "type": "list",
    "default": ["newtag"],
    "doc": "Tags to add to elements in the resulting offset region."
  },
  {
    "pointer": "/offset_output_tags/*",
    "type": "string",
    "doc": "A tag."
  },
  {
    "pointer": "/protected_tags",
    "type": "list",
    "default": [],
    "doc": "Only relevant if overwrite_tags=true. Set of tags that will not be overwritten by the offset."
  },
  {
    "pointer": "/protected_tags/*",
    "type": "string",
    "doc": "A tag."
  },
  {
    "pointer": "/overwrite_tags",
    "type": "bool",
    "default": false,
    "doc": "If true, offset_output_tag overwrites existing tags in the offset."
  },
  {
    "pointer": "/offset_in",
    "type": "bool",
    "default": false,
    "doc": "Only relevant for single body mode. Whether to create offset inside body"
  },
  {
    "pointer": "/offset_out",
    "type": "bool",
    "default": true,
    "doc": "Only relevant for single body mode. Whether to create offset outside body"
  },
  {
    "pointer": "/respect_all_topologies",
    "type": "bool",
    "default": false,
    "doc": "If true, the topology (after offset initialization) of every tag is respected as if the offset was to replace all tags. If false, only the topology of the offset is respected."
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
