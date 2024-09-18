#pragma once
#include <nlohmann/json.hpp>


namespace wmtk::applications::procedural {
nlohmann::json spec = R"(
[
  {
    "pointer": "/",
    "required": [
      "name",
      "type",
      "fan"
    ],
    "type": "object",
    "type_name": "triangle_fan"
  },
  {
    "pointer": "/",
    "required": [
      "name",
      "type",
      "disk"
    ],
    "type": "object",
    "type_name": "disk"
  },
  {
    "pointer": "/",
    "required": [
      "name",
      "type",
      "grid"
    ],
    "type": "object",
    "type_name": "grid"
  },
  {
    "pointer": "/name",
    "type": "string"
  },
  {
    "doc": "The type of mesh to generate. The settings are assumed to be for the right type",
    "options": [
      "grid",
      "triangle_fan",
      "disk"
    ],
    "pointer": "/type",
    "type": "string"
  },
  {
    "doc": "Settings for a regular grid",
    "optional": [
      "coordinates",
      "tiling",
      "cycles"
    ],
    "pointer": "/grid",
    "required": [
      "dimensions"
    ],
    "type": "object"
  },
  {
    "default": "default",
    "doc": "tiling method used to generate the grid. Default is determined by the dimension (diagonal or freudenthal) ",
    "options": [
      "BCC",
      "freudenthal",
      "diagonal"
    ],
    "pointer": "/grid/tiling",
    "type": "string"
  },
  {
    "doc": "The number of elements along each axis of the grid",
    "max": 3,
    "min": 2,
    "pointer": "/grid/dimensions",
    "type": "list"
  },
  {
    "pointer": "/grid/dimensions/*",
    "type": "int"
  },
  {
    "default": [
      false,
      false,
      false
    ],
    "doc": "Whether this axis shoudl be cyclic (currently unimplemented) ",
    "max": 3,
    "min": 2,
    "pointer": "/grid/cycles",
    "type": "list"
  },
  {
    "pointer": "/grid/cycles/*",
    "type": "bool"
  },
  {
    "default": null,
    "optional": [
      "name",
      "spacing"
    ],
    "pointer": "/grid/coordinates",
    "type": "object"
  },
  {
    "default": "",
    "pointer": "/grid/coordinates/name",
    "type": "string"
  },
  {
    "default": [
      0,
      0,
      0
    ],
    "doc": "The spacing between any node in the grid in each axis.",
    "max": 3,
    "min": 2,
    "pointer": "/grid/coordinates/spacing",
    "type": "list"
  },
  {
    "pointer": "/grid/coordinates/spacing/*",
    "type": "float"
  },
  {
    "doc": "Settings for a fan mesh (a single interior vertex surrounded by triangles like a pie) ",
    "pointer": "/fan",
    "required": [
      "size",
      "coordinates"
    ],
    "type": "object"
  },
  {
    "doc": "The number of triangles used. Assumes at least 2 triangles for now, but this should change later on",
    "min": 2,
    "pointer": "/fan/size",
    "type": "int"
  },
  {
    "doc": "The coordinates of the triangle fan. uses (center + radius * (cos(degree),sin(degree))) ",
    "optional": [
      "center"
    ],
    "pointer": "/fan/coordinates",
    "required": [
      "name",
      "degrees",
      "radius"
    ],
    "type": "object"
  },
  {
    "pointer": "/fan/coordinates/name",
    "type": "string"
  },
  {
    "default": [
      0,
      0
    ],
    "doc": "The coordinates for the center",
    "max": 2,
    "min": 2,
    "pointer": "/fan/coordinates/center",
    "type": "list"
  },
  {
    "pointer": "/fan/coordinates/center/*",
    "type": "float"
  },
  {
    "pointer": "/fan/coordinates/radius",
    "type": "float"
  },
  {
    "doc": "The range of degrees the fan should use",
    "max": 2,
    "min": 2,
    "pointer": "/fan/coordinates/degrees",
    "type": "list"
  },
  {
    "doc": "The max degree should use (assumes the range starts at 0) ",
    "pointer": "/fan/coordinates/degrees",
    "type": "float"
  },
  {
    "pointer": "/fan/coordinates/degrees/*",
    "type": "float"
  },
  {
    "doc": "Settings for a disk mesh (a single interior vertex surrounded by triangles like a pie) ",
    "pointer": "/disk",
    "required": [
      "size",
      "coordinates"
    ],
    "type": "object"
  },
  {
    "doc": "The number of triangles used. Assumes at least 2 triangles for now, but this should change later on",
    "min": 2,
    "pointer": "/disk/size",
    "type": "int"
  },
  {
    "doc": "The coordinates of the triangle fan. uses (center + radius * (cos(degree),sin(degree))) ",
    "optional": [
      "degree_offset",
      "center"
    ],
    "pointer": "/disk/coordinates",
    "required": [
      "name",
      "radius"
    ],
    "type": "object"
  },
  {
    "pointer": "/disk/coordinates/name",
    "type": "string"
  },
  {
    "pointer": "/disk/coordinates/radius",
    "type": "float"
  },
  {
    "default": 0,
    "doc": "What angle (in degrees) the disk starts at",
    "pointer": "/disk/coordinates/degree_offset",
    "type": "float"
  },
  {
    "default": [
      0,
      0
    ],
    "max": 3,
    "min": 2,
    "pointer": "/disk/coordinates/center",
    "type": "list"
  },
  {
    "pointer": "/disk/coordinates/center/*",
    "type": "float"
  }
]
    )"_json;
}

