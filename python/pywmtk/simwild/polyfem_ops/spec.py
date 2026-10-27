"""Minimal validator for the wmtk-style JSON spec rule lists (same format as
simwild_spec.json): pointer/type/required/optional/default/options/min/max.
Fills defaults for absent optional keys and rejects unmatched pointers with
the same "No rule matched" wording as the C++ jse."""
import json
from importlib import resources

_TYPES = {
    "string": str,
    "bool": bool,
    "int": int,
    "float": (int, float),
    "list": (list, tuple),
    "object": dict,
}


def load_spec(op_name: str) -> list:
    """Load the spec.json packaged with simwild.polyfem_ops.<op_name>."""
    with resources.files(f"simwild.polyfem_ops.{op_name}").joinpath(
            "spec.json").open() as f:
        return json.load(f)


def _rule_for(rules, pointer):
    """Component-wise pointer match where any rule component may be `*`;
    the most specific matching rule (fewest wildcards) wins."""
    parts = pointer.split("/")
    best, best_score = None, -1
    for r in rules:
        rp = r["pointer"].split("/")
        if len(rp) != len(parts):
            continue
        if all(a == b or a == "*" for a, b in zip(rp, parts)):
            score = sum(a != "*" for a in rp)
            if score > best_score:
                best, best_score = r, score
    return best


def _check(rules, pointer, value):
    rule = _rule_for(rules, pointer)
    if rule is None:
        raise ValueError(f'No rule matched for "{pointer}"')
    typ = rule["type"]
    allowed = tuple(t for name in ([typ] if isinstance(typ, str) else typ)
                    for t in ([_TYPES[name]] if not isinstance(_TYPES[name], tuple)
                              else list(_TYPES[name])))
    if not isinstance(value, allowed) or (isinstance(value, bool)
                                          and bool not in allowed):
        raise ValueError(
            f'"{pointer}": expected {typ}, got {type(value).__name__} ({value!r})')
    if "options" in rule and value not in rule["options"]:
        raise ValueError(f'"{pointer}": {value!r} not in {rule["options"]}')
    if isinstance(value, (list, tuple)):
        if "min" in rule and len(value) < rule["min"]:
            raise ValueError(f'"{pointer}": needs >= {rule["min"]} entries')
        if "max" in rule and len(value) > rule["max"]:
            raise ValueError(f'"{pointer}": needs <= {rule["max"]} entries')
        for i, v in enumerate(value):
            _check(rules, f"{pointer}/{i}", v)
    elif isinstance(value, dict) and pointer != "/":
        # Object rules may pin their key set via required/optional; without
        # them any key is allowed (matched through * rules, e.g. amips_weights).
        req, opt = rule.get("required"), rule.get("optional")
        if req is not None or opt is not None:
            req, opt = req or [], opt or []
            missing = [k for k in req if k not in value]
            if missing:
                raise ValueError(f'"{pointer}": missing required key(s) {missing}')
            unknown = [k for k in value if k not in req and k not in opt]
            if unknown:
                raise ValueError(
                    f'"{pointer}": unknown key(s) {unknown}; '
                    f'allowed: {sorted(req + opt)}')
        for k, v in value.items():
            _check(rules, f"{pointer}/{k}", v)


def validate(rules: list, params: dict) -> dict:
    """Validate `params` against the rule list; returns a new dict with
    top-level defaults filled in."""
    root = _rule_for(rules, "/")
    required = root.get("required", [])
    optional = root.get("optional", [])
    missing = [k for k in required if k not in params]
    if missing:
        raise ValueError(f"missing required parameter(s): {missing}")
    unknown = [k for k in params if k not in required and k not in optional]
    if unknown:
        raise ValueError(
            f"unknown parameter(s) {unknown}; allowed: {sorted(required + optional)}")

    out = dict(params)
    for key in required + optional:
        if key in out:
            _check(rules, f"/{key}", out[key])
        else:
            rule = _rule_for(rules, f"/{key}")
            if rule is not None and "default" in rule:
                out[key] = rule["default"]
    return out
