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
      "tag_name",
      "sep_tag_vals",
      "offset_tag_val",
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
    "pointer": "/save_vtu",
    "type": "bool",
    "default": "false",
    "doc": "Save .vtu of output mesh"
  },
  {
    "pointer": "/tag_name",
    "type": "string",
    "default": "tag_0",
    "doc": "Tetwise tag name to determine boundaries for offset"
  },
  {
    "pointer": "/sep_tag_vals",
    "type": "list",
    "default": [
      0,
      1
    ],
    "min": 2,
    "doc": "Tag values whose pairwise boundaries will be offset"
  },
  {
    "pointer": "/sep_tag_vals/*",
    "type": "int"
  },
  {
    "pointer": "/offset_tag_val",
    "type": "int",
    "default": 0,
    "doc": "Tag value to fill offset region"
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