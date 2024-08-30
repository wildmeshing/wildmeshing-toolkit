namespace {

nlohmann::json input_spec = R"(
[
  {
    "pointer": "/",
    "type": "object",
    "required": ["file"],
    "optional": ["ignore_z", "tetrahedron_attributes"]
  },
  {
    "pointer": "/file",
    "type": "string"
  },
  {
    "pointer": "/ignore_z",
    "type": "bool",
    "default": false
  },
  {
    "pointer": "/tetrahedron_attributes",
    "type": "list",
    "default": [],
    "doc": "all attributes on tetrahedra that should be loaded from the msh file"
  },
  {
    "pointer": "/tetrahedron_attributes/*",
    "type": "string"
  }
]
)"_json;

}