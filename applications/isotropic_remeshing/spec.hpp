#pragma once
#include <nlohmann/json.hpp>

namespace wmtk::applications::isotropic_remeshing {
nlohmann::json spec = R"(
[
  {
    "pointer": "/",
    "type": "object",
    "required": [
      "input",
      "output",
      "attributes",
      "iterations"
      ],
    "optional": [
      "imported_attributes",
      "name_spec",
      "old_mode",
      "ignore_z",
      "tet_attributes"
    ],
    "pointer": "/input",
    "type": "object"
  },
  {
    "default": {},
    "pointer": "/input/name_spec",
    "type": "object"
  },
  {
    "default": [],
    "pointer": "/input/imported_attributes",
    "type": "list"
  },
  {
    "pointer": "/input/imported_attributes/*",
    "type": "list"
  },
  {
    "pointer": "/input/imported_attributes/*/*",
    "type": "string"
  },
  {
    "default": false,
    "pointer": "/input/old_mode",
    "type": "bool"
  },
  {
    "default": false,
    "pointer": "/input/ignore_z",
    "type": "bool"
  },
  {
    "default": [],
    "pointer": "/input/tet_attributes",
    "type": "list"
  },
  {
    "pointer": "/input/tet_attributes/*",
    "type": "string"
  },
  {
    "doc": "output mesh",
    "pointer": "/output",
    "type": "string"
  },
  {
    "pointer": "/position_attribute",
    "type": "object"
  },
  {
    "default": {},
    "pointer": "/inversion_position_attribute",
    "type": "object"
  },
  {
    "default": {},
    "pointer": "/other_position_attributes",
    "type": "list"
  },
  {
    "default": [],
    "doc": "all attributes that are not deleted by the component but also not required",
    "pointer": "/pass_through",
    "type": "list"
  },
  {
    "pointer": "/pass_through/*",
    "type": "list"
  },
  {
    "default": "",
    "pointer": "/pass_through/*/*",
    "type": "string"
  },
  {
    "pointer": "/pass_through/*/*",
    "required": [
      "name",
      "mesh"
    ],
    "type": "object"
  },
  {
    "pointer": "/pass_through/*/*/name",
    "type": "string"
  },
  {
    "pointer": "/pass_through/*/*/mesh",
    "type": "string"
  },
  {
    "doc": "number of iterations that should be performed",
    "pointer": "/iterations",
    "type": "int"
  },
  {
    "default": -1,
    "pointer": "/length_abs",
    "type": "float"
  },
  {
    "default": -1,
    "pointer": "/length_rel",
    "type": "float"
  },
  {
    "default": true,
    "pointer": "/lock_boundary",
    "type": "bool"
  },
  {
    "default": false,
    "pointer": "/use_for_periodic",
    "type": "bool"
  },
  {
    "default": false,
    "pointer": "/dont_disable_split",
    "type": "bool"
  },
  {
    "default": false,
    "pointer": "/fix_uv_seam",
    "type": "bool"
  }
]
)"_json;
}
