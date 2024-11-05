namespace {

nlohmann::json tw_simplification_spec = R"(
[
  {
    "pointer": "/",
    "type": "object",
    "required": [
      "input",
      "output",
      "main_eps"
    ],
    "optional": [
      "root",
      "report",
      "duplicate_tol",
      "sample_envelope",
      "relative"
    ]
  },
  {
    "pointer": "/input",
    "type": "string"
  },
  {
    "pointer": "/output",
    "type": "string"
  },
  {
    "pointer": "/main_eps",
    "type": "float",
    "doc": "main esp tolerance"
  },
  {
    "pointer": "/duplicate_tol",
    "type": "float",
    "default": -1,
    "doc": "tolerance for removing duplicate vertices"
  },
  {
    "pointer": "/relative",
    "type": "bool",
    "default": true,
    "doc": "relative or absolute envelope size and duplicate_tol"
  },
  {
    "pointer": "/sample_envelope",
    "type": "bool",
    "default": false,
    "doc": "use sampling envelope"
  },
  {
    "pointer": "/report",
    "type": "string",
    "default": ""
},
{
    "pointer": "/root",
    "type": "string",
    "default": ""
}
]
)"_json;

}