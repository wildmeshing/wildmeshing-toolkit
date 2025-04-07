namespace {

nlohmann::json simplicial_embedding_spec = R"(
[
  {
    "pointer": "/",
    "type": "object",
    "required": ["input", "tag_name", "output"],
    "optional": [
      "tag_value",
      "tag_default_value",
      "embed_interfaces",
      "root",
      "report"
    ]
  },
  {
    "pointer": "/input",
    "type": "string",
    "doc": "The input mesh as .msh."
  },
  {
    "pointer": "/tag_name",
    "type": "string",
    "doc": "The tag on tetrahedra to distinguish different regions. Tags must be integer numbers."
  },
  {
    "pointer": "/tag_value",
    "type": "int",
    "default": 0,
    "doc": "The tag representing the region that should be simplicially embedded. This value is ignored if `embed_interfaces` is true."
  },
  {
    "pointer": "/tag_default_value",
    "type": "int",
    "default": -1,
    "doc": "This value must be different from the `tag_value`."
  },
  {
    "pointer": "/embed_interfaces",
    "type": "bool",
    "default": false,
    "doc": "If true, all triangles on the boundary and in between tetrahedra will be simplicially embedded."
  },
  {
    "pointer": "/output",
    "type": "string",
    "doc": "The output file. The extension should be either .msh or nothing. The latter outputs .vtu files"
  },
  {
    "pointer": "/report",
    "type": "string",
    "default": "",
    "doc": "Write a report to a .json file. Mostly for integration testing."
  },
  {
    "pointer": "/root",
    "type": "string",
    "default": "",
    "doc": "The root folder for input files. All paths are considered relative to the .json spec file."
  }
]
)"_json;

}