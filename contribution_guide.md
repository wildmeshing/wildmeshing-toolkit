# Contribution Guide

## General Advises

- Use self explaining variable names. If the existence of a variable is not obvious, write a comment explaining why it is there.
- Write self explaining code. In case of doubt explain what the code does and why it was implemented that way. The same holds for tests.
- Do not define global variables in header files.
- Classes should be written in a file that has the same name as the class. For example, the class `TriMesh` is in the file _TriMesh.hpp_ and its implementation in _TriMesh.cpp_.
- Use `const` as much as possible.
- Avoid complex nested expressions by splitting them up over multiple lines. Example:
  ```
  // hard to read
  CHECK(fv_accessor.vector_attribute(new_fids[0][0])[0] == 0);
  // better
  const long f0 = new_fids[0][0];
  const auto fv0 = fv_accessor.vector_attribute(f0);
  CHECK(fv0[0] == 0);
  ```
- Only use `auto` to abbreviate lengthy types like `std::vector<std::array<double,3>>`. If your variable is just a `Tuple` or a `Simplex`, avoid `auto`.

## Naming Conventions

- Variables and functions are lower case with underscore, only class names use upper camel case. Examples:
  - `my_function`
  - `my_variable_name`
  - `MyClass`
- `const` or `constexpr` variables may be fully capitalized, e.g. `MY_CONSTANT`.
- All member variables start with `m_`, e.g. `m_my_member_variable`.

## Tests

- Test cases must not contain spaces, use underscore instead, e.g. `my_test_case`.
- Avoid the word _test_. That's like calling a function `function`.
- Use CHECK instead of REQUIRE whenever it makes sense. If an unexpected result of a REQUIRE does not cause the program to crash, it should be a CHECK not a REQUIRE.
- Assign groups to test cases. Examples:
  - 2D / 3D
  - operations
  - simplicial_complex
- If you add a test that cannot pass yet (because the tested code is not yet implemented), add a `[.]` tag to exclude it from running.
