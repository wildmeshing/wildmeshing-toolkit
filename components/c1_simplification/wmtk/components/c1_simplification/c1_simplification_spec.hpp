#pragma once
#include <nlohmann/json.hpp>
namespace {
nlohmann::json c1_simplification_spec = R"([
  {
    "pointer": "/",
    "type": "object",
    "required": [
      "application",
      "surface_mesh",
      "tet_mesh",
      "uv_mesh",
      "dofs",
      "cones"
    ],
    "optional": [
      "output",
      "threshold",
      "log_level"
    ]
  },
  {
    "pointer": "/application",
    "type": "string",
    "options": [
      "c1_simplification"
    ],
    "doc": "Application name must be c1_simplification."
  },
  {
    "pointer": "/surface_mesh",
    "type": "string",
    "doc": "surface mesh file name"
  },
  {
    "pointer": "/tet_mesh",
    "type": "string",
    "doc": "tet mesh file name"
  },
  {
    "pointer": "/uv_mesh",
    "type": "string",
    "doc": "uv mesh file name"
  },
  {
    "pointer": "/dofs",
    "type": "string",
    "doc": "dofs file name"
  },
  {
    "pointer": "/cones",
    "type": "string",
    "doc": "cone vids file name"
  },
  {
    "pointer": "/output",
    "type": "string",
    "default": "out",
    "doc": "Output file name (without extension)."
  },
  {
    "pointer": "/log_level",
    "type": "int",
    "default": 2,
    "doc": "wmtk logger level."
  },
  {
    "pointer": "/threshold",
    "type": "float",
    "default": 0.1,
    "doc": "threshold of deviation"
  }
]
)"_json;
}