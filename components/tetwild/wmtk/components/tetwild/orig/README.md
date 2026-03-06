# TetWild (original)

This folder contains a copy of the original TetWild mesh refinement code. It does not contain the embedding part. The CGAL and Geogram dependencies were replaced by WMTK functionality.

The point of entry is the `MeshRefinement` class:

```
orig::MeshRefinement legacy_tetwild(*env, *env_b, args, state);

// set state ...

legacy_tetwild.prepareData();
legacy_tetwild.refine(); // the actual tetwild
```

## Difference WMTK and original TetWild

- Priority queue is slightly different as one of them is using the tbb::queue and the other the std::queue. Also, the queue elements might be added in a different order. That could be fixed if really necessary but should not have a large impact overall.
- Collapse: The number of successful operations is very similar in both.
- Split is the same
- Swap has major differences
  - 23 swap does not exist in TW but in WMTK
  - 23 swap does not get all faces but all edges to initialize the queue
  - 56 swap is missing in WMTK
