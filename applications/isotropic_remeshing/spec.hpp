#pragma once
#include <nlohmann/json.hpp>

namespace wmtk::applications::isometric_remeshing {
nlohmann::json spec = R"(
[
  {
    "optional": [
      "pass_through",
      "length_abs",
      "length_rel",
      "lock_boundary",
      "use_for_periodic",
      "dont_disable_split"
    ],
    "pointer": "/",
    "required": [
      "input",
      "output",
      "attributes",
      "iterations"
    ],
    "type": "object"
  },
  {
    "optional": [
      "imported_attributes",
      "name_spec",
      "old_mode",
      "ignore_z",
      "tet_attributes"
    ],
    "pointer": "/input",
    "required": [
      "file"
    ],
    "type": "object"
  },
  {
    "pointer": "/input/file",
    "type": "string"
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
    "doc": "all attributes required for this component",
    "optional": [
      "inversion_position",
      "other_positions",
      "update_other_positions"
    ],
    "pointer": "/attributes",
    "required": [
      "position"
    ],
    "type": "object"
  },
  {
    "pointer": "/attributes/position",
    "type": "list"
  },
  {
    "default": "",
    "pointer": "/attributes/position/*",
    "type": "string"
  },
  {
    "pointer": "/attributes/position/*",
    "required": [
      "name",
      "mesh"
    ],
    "type": "object"
  },
  {
    "pointer": "/attributes/position/*/name",
    "type": "string"
  },
  {
    "pointer": "/attributes/position/*/mesh",
    "type": "string"
  },
  {
    "default": [],
    "pointer": "/attributes/inversion_position",
    "type": "list"
  },
  {
    "default": "",
    "pointer": "/attributes/inversion_position/*",
    "type": "string"
  },
  {
    "pointer": "/attributes/inversion_position/*",
    "required": [
      "name",
      "mesh"
    ],
    "type": "object"
  },
  {
    "pointer": "/attributes/inversion_position/*/name",
    "type": "string"
  },
  {
    "pointer": "/attributes/inversion_position/*/mesh",
    "type": "string"
  },
  {
    "default": [],
    "pointer": "/attributes/other_positions",
    "type": "list"
  },
  {
    "default": "",
    "pointer": "/attributes/other_positions/*",
    "type": "string"
  },
  {
    "pointer": "/attributes/other_positions/*",
    "required": [
      "name",
      "mesh"
    ],
    "type": "object"
  },
  {
    "pointer": "/attributes/other_positions/*/name",
    "type": "string"
  },
  {
    "pointer": "/attributes/other_positions/*/mesh",
    "type": "string"
  },
  {
    "default": false,
    "doc": "Use attribute update to sync other positions",
    "pointer": "/attributes/update_other_positions",
    "type": "bool"
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
