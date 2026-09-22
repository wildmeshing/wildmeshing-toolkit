"""Selections in Python: the wmtk expression grammar (through the wildmeshing
bindings) and the normalization and id assignment of region/filter
selections, which polyfem_sim uses. Loading a tagged mesh and picking the
selected faces happen in the C++ component polyfem_ops.

A selection is the boundary of a region, optionally filtered:
{"region": expr, "filter": expr?, "id": int?} (bare string = region). A
face is selected iff its inside cell satisfies `region`, the outside does
not, and — when given — the outside satisfies `filter`. Expressions follow
the wmtk grammar (&, |, !, parentheses; names = physical groups).
Orientation: outward from the region. Interior faces only; the domain
boundary is never selected.
"""


# ---------------------------------------------------------------------------
# Tag expressions — delegated to the C++ parser via the wildmeshing bindings
# (see app/pywildmeshing/pywildmeshing.cpp::Expression). One grammar, one
# implementation; only the identifier lexer below lives in Python.
# ---------------------------------------------------------------------------

def _expr_atoms(expr: str):
    """Identifiers appearing in an expression (lexing only, no grammar)."""
    out, cur = set(), ""
    for c in expr:
        if c.isalnum() or c in "_-":
            cur += c
        else:
            if cur:
                out.add(cur)
            cur = ""
    if cur:
        out.add(cur)
    return out


def parse_expression(expr: str):
    """Compile a wmtk-grammar tag expression through the C++ parser.
    Returns (predicate over a set of names, set of names referenced)."""
    if not isinstance(expr, str):
        raise ValueError(
            f"selection expression must be a string like 'tag_0 | tag_1', "
            f"got {expr!r}")
    from wildmeshing import Expression
    names = _expr_atoms(expr) - {"_"}
    try:
        compiled = Expression(expr, sorted(names))
    except RuntimeError as e:
        raise ValueError(f"selection {expr!r}: {e}") from None
    return (lambda tags: compiled.eval(set(tags))), names


# ---------------------------------------------------------------------------
# Interface selection: boundary of a region, optionally filtered
# ---------------------------------------------------------------------------

def normalize_selection(spec):
    """Normalize one selection spec to (region_expr, filter_expr|None, id|None).
    Accepts a bare region string or {"region": str, "filter": str, "id": int}."""
    if isinstance(spec, str):
        return spec, None, None
    if isinstance(spec, dict) and "region" in spec:
        extra = set(spec) - {"region", "filter", "id"}
        if extra:
            raise ValueError(f"selection {spec!r}: unknown key(s) {sorted(extra)}")
        sid = spec.get("id")
        filt = spec.get("filter")
        if not isinstance(spec["region"], str) or (
                filt is not None and not isinstance(filt, str)):
            raise ValueError(f"selection {spec!r}: region/filter must be strings")
        return spec["region"], filt, (None if sid is None else int(sid))
    raise ValueError(
        f"a selection is a region expression string or {{'region': str, "
        f"'filter': str, 'id': int}}, got {spec!r} (pairs and 'a & b' "
        f"conjunction selections are no longer supported)")


def assign_selection_ids(selections, require_ids=False):
    """Normalize + dedupe selections and assign ids.

    Identical (region, filter) specs collapse to ONE selection with one id
    (conflicting explicit ids on the same selection raise; distinct
    selections may share an explicit id to form one body). Auto ids are
    assigned sequentially, never colliding with explicit ones.

    Returns (unique, ids_per_input): unique = [{"region", "filter", "id"}]
    in first-appearance order; ids_per_input = id for each input spec.
    """
    specs = [normalize_selection(x) for x in selections]
    if require_ids and any(sid is None for _, _, sid in specs):
        raise ValueError("every selection needs an explicit 'id' here")

    by_key: dict = {}
    order = []
    for region, filt, sid in specs:
        key = (region.strip(), filt.strip() if filt is not None else None)
        if key not in by_key:
            by_key[key] = sid
            order.append(key)
        elif sid is not None:
            if by_key[key] is not None and by_key[key] != sid:
                raise ValueError(
                    f"selection region={key[0]!r} filter={key[1]!r} given "
                    f"conflicting ids {by_key[key]} and {sid}")
            by_key[key] = sid

    reserved = {sid for sid in by_key.values() if sid is not None}
    next_id = 1
    for key in order:
        if by_key[key] is None:
            while next_id in reserved:
                next_id += 1
            by_key[key] = next_id
            next_id += 1

    unique = [{"region": k[0], "filter": k[1], "id": by_key[k]} for k in order]
    ids_per_input = [
        by_key[(r.strip(), f.strip() if f is not None else None)]
        for r, f, _ in specs]
    return unique, ids_per_input
