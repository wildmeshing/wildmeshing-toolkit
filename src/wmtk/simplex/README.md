# Simplex

The classes and functions in this class represent simplices (vertex, edge, triangle, tetrahedron) and provide functions for navigation.

#### Simplex Classes

By now there are several classes for representing a simplex. They use different information to represent simplices:

- `Simplex` (deprecated) - contains a Tuple and PrimitiveType
- `IdSimplex` - contains a global simplex ID and PrimitiveType
- `NavigatableSimplex` - extends `IdSimplex` by adding a Tuple. This should replace the deprecated `Simplex` at some point
- `RawSimplex` - contains a vector of vertex IDs. This is mostly for debugging and should not be used as it is very slow.

#### SimplexCollection

A container for simplices. It has some basic functionality to compute the intersection or union of collections.

#### Navigation

By now, for most of the navigation functions (open star, closed star, link, etc.) an iterable version exists. This one should be preferred in almost all cases as it is more efficient and avoids buffering simplices in a `SimplexCollection`.
