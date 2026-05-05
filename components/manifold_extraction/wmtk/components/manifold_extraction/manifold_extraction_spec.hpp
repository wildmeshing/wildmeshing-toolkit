#pragma once
#include <nlohmann/json.hpp>
namespace {

nlohmann::json manifold_extraction_spec = R"(
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
      "in_tag",
      "manifold_union",
      "replace_tag",
      "DEBUG_output",
      "write_surface"
    ]
  },
  {
    "pointer": "/application",
    "type": "string",
    "options": [
      "manifold_extraction"
    ],
    "doc": "Application name must be manifold_extraction."
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
    "pointer": "/in_tag",
    "type": "list",
    "default": [0],
    "min": 1,
    "doc": "Tag set to considered inside the surface."
  },
  {
    "pointer": "/in_tag/*",
    "type": "int",
    "doc": "A tag."
  },
  {
    "pointer": "/manifold_union",
    "type": "bool",
    "default": true,
    "doc": "If true, offsets are unioned with mesh, otherwise subtracted from mesh."
  },
  {
    "pointer": "/replace_tag",
    "type": "list",
    "default": [],
    "doc": "Only relevant for subtract mode (manifold_union=false). Tag set to fill subtracted regions with. Ambient (empty) by default."
  },
  {
    "pointer": "/DEBUG_output",
    "type": "bool",
    "default": false,
    "doc": "Write the tet mesh as out_{}.vtu after every operation."
  },
  {
    "pointer": "/write_surface",
    "type": "bool",
    "default": false,
    "doc": "Write the output surface (OBJ) after manifold extraction."
  }
]
)"_json;

}