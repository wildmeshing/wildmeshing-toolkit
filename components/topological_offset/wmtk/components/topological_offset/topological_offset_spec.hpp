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
      "tag_label",
      "sep_tags",
      "fill_tag",
      "manifold_mode",
      "manifold_union",
      "num_threads",
      "write_vtu",
      "log_file",
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
    "pointer": "/tag_label",
    "type": "string",
    "default": "tag_0",
    "doc": "Tetwise tag label to identify components. If manifold extraction mode, cells assumed to have values 0/1."
  },
  {
    "pointer": "/sep_tags",
    "type": "list",
    "default": [0, 1],
    "doc": "List of tag values for components to pairwise separate.",
    "min": 2
  },
  {
    "pointer": "/sep_tags/*",
    "type": "int",
    "doc": "Tag value for component to be separated."
  },
  {
    "pointer": "/fill_tag",
    "type": "int",
    "default": 0,
    "doc": "Label for component to fill in offset"
  },
  {
    "pointer": "/manifold_mode",
    "type": "bool",
    "default": false,
    "doc": "Manifold extraction mode. If true, input is taken as all non-manifold simplices."
  },
  {
    "pointer": "/manifold_union",
    "type": "bool",
    "default": true,
    "doc": "Only relevant in manifold extraction mode. If true, offsets are unioned with mesh, otherwise subtracted from mesh."
  },
  {
    "pointer": "/num_threads",
    "type": "int",
    "default": 0,
    "doc": "Number of threads used by the application."
  },
  {
    "pointer": "/write_vtu",
    "type": "bool",
    "default": false,
    "doc": "Write not just MSH but also VTU output."
  },
  {
    "pointer": "/log_file",
    "type": "string",
    "default": "",
    "doc": "Logs are not just printed on the terminal but also saved in this file."
  },
  {
    "pointer": "/DEBUG_output",
    "type": "bool",
    "default": false,
    "doc": "Write the mesh as out_{}.vtu after every operation."
  }
]

)"_json;

}