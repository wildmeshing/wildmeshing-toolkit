# Expression Parser for Tags

Tags are given by the user as strings, e.g., `"tag_0"`. Internally, tags are stored as `std::set<int64_t>`.
We have maps in the mesh classes that convert from strings to integers:

```
std::map<int64_t, std::string> m_tag_id_to_name;
std::map<std::string, int64_t> m_tag_name_to_id;
```

A region in the mesh can be described through expressions of these tags, e.g.:

```
"(tag_0 | tag_1) & !tag_2"
```

This folder contains code to parse such expressions.
The following syntax tokens are allowed:

- parantheses `(` and `)`
- and `&`
- or `|`
- not `!`
- tag identifiers, e.g., `tag_0`
- ambient tag identifier `_`

Priority in operations are:

1. `!`
2. `&`
3. `|`

For example, `A | B & C` would evaluate as `A | (B & C)`.
