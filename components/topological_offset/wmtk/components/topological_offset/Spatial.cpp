// INTENTIONALLY EMPTY.
//
// This held the conservative offset-growth predicates -- circle/sphere subdivision against the
// Euclidean distance field, and the two topology-consistency checks that gated which cells growth
// was allowed to absorb. Both dimensions dropped the growth pass: construction is now simplicial
// embedding plus midpoint marching, and carrying the offset out to target_distance is entirely
// the optimization phase's job. The 3D half went with that change; this is the 2D half.
//
// Circle.hpp / Sphere.hpp remain: they are standalone geometric primitives with their own unit
// tests, and nothing in the pipeline includes them any more.
