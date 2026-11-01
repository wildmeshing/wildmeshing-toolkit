# Citing

If you use the toolkit in academic work, please cite the paper that describes it:

```bibtex
@article{jiang2022declarative,
  title   = {Declarative Specification for Unstructured Mesh Editing Algorithms},
  author  = {Jiang, Zhongshi and Dai, Jiacheng and Hu, Yixin and Zhou, Yunfan and Dumas, Jeremie
             and Zhou, Qingnan and Bajwa, Gurkirat Singh and Zorin, Denis and Panozzo, Daniele
             and Schneider, Teseo},
  journal = {ACM Transactions on Graphics},
  volume  = {41},
  number  = {6},
  pages   = {1--14},
  year    = {2022},
  doi     = {10.1145/3550454.3555513}
}
```

The API has changed since the paper; the version it describes is tag
[`1.0`](https://github.com/wildmeshing/wildmeshing-toolkit/releases/tag/1.0).

And the papers of the algorithms you use:

| Component | Paper |
|---|---|
| `tetwild` | Hu, Zhou, Gao, Jacobson, Zorin, Panozzo. [Tetrahedral Meshing in the Wild](https://cims.nyu.edu/gcl/papers/2018-TetWild.pdf). ACM TOG 37(4), 2018. And Hu, Schneider, Wang, Zorin, Panozzo. [Fast Tetrahedral Meshing in the Wild](https://arxiv.org/abs/1908.03581). ACM TOG 39(4), 2020. |
| `triwild` | Hu, Schneider, Gao, Zhou, Jacobson, Zorin, Panozzo. [TriWild: Robust Triangulation with Curve Constraints](https://cims.nyu.edu/gcl/papers/2019-TriWild.pdf). ACM TOG 38(4), 2019. |
| envelopes | Wang, Schneider, Hu, Attene, Panozzo. [Exact and Efficient Polyhedral Envelope Containment Check](https://cims.nyu.edu/gcl/papers/2020-Fast-Envelope.pdf). ACM TOG 39(4), 2020. |
| `topological_offset` | [Topological Offsets](https://dl.acm.org/doi/10.1145/3731157). ACM TOG, 2025. |
| `tracking` | [arXiv 2605.30744](https://arxiv.org/abs/2605.30744). |
| `qslim` | Garland, Heckbert. [Surface Simplification Using Quadric Error Metrics](https://dl.acm.org/doi/10.1145/258734.258849). SIGGRAPH 1997. |
| `shortest_edge_collapse` | Hoppe. [Progressive Meshes](https://hhoppe.com/pm.pdf). SIGGRAPH 1996. |
| `isotropic_remeshing` | Botsch, Kobbelt. [A Remeshing Approach to Multiresolution Modeling](https://ls7-gv.cs.tu-dortmund.de/downloads/publications/2004/sgp04.pdf). SGP 2004. |
