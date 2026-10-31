"""
Python bindings for the Wildmeshing-Toolkit
"""
from __future__ import annotations
import collections.abc
import numpy
import os
import typing
__all__: list[str] = ['CurveTracking', 'Expression', 'Tracking', 'VolumeTracking', 'c1_simplification', 'components', 'isotropic_remeshing', 'manifold_extraction', 'prismatic_mesh', 'qslim', 'read_history', 'read_mesh', 'shortest_edge_collapse', 'simwild', 'spec', 'tetwild', 'topological_offset', 'tracking', 'triwild', 'wildmeshing', 'write_history', 'write_mesh']
class Expression:
    def __init__(self, expr: str, known_names: collections.abc.Sequence[str]) -> None:
        """
        Parse a tag expression (names, &, |, !, parentheses, _) against the given known names.
        """
    def eval(self, tags: collections.abc.Set[str]) -> bool:
        """
        Evaluate against a set of tag names.
        """
    def has_unknown_names(self) -> bool:
        """
        True if the expression references names outside known_names.
        """
class Tracking:
    """
    The map between a surface application's input and its output, both ways, built from the history it recorded (its history_output): the bytes a memory call returns as report['history'], or Tracking.load(path) of the file the file route writes. Points on the input are named by triangles of the input files, or, for a history that starts at a tet mesh (input_by_vertices), by three of its vertices; points on the output by triangles of a surface output, or by three vertices of a tet output (output_by_vertices). Mapping never changes the object.
    """
    @staticmethod
    def load(path: os.PathLike | str | bytes, smoothing: str = 'flatten', scaffold: str = 'box', collapse_patch: str = 'both_stars') -> Tracking:
        """
        Build the map from a history file, as from its bytes (wm.read_history).
        """
    def __init__(self, history: bytes, smoothing: str = 'flatten', scaffold: str = 'box', collapse_patch: str = 'both_stars') -> None:
        """
        Build the map from the bytes of a surface history. smoothing ('flatten' or 'barycentric'), scaffold ('box' or 'ring') and collapse_patch ('both_stars' or 'removed_star') as the tracking component's options.
        """
    def to_input(self, ids: typing.Any, bary: typing.Any, warnings: bool = False) -> tuple:
        """
        Points on the output, to the input: the inverse map. Vertices may be given in any order, the coordinates following them. ids: int64 (n,), a face (segment) per point, or (n, 3) -- (n, 2) on curves -- vertices where the side names points by the vertices of a mesh (output_by_vertices, input_by_vertices); -1 for none. bary: float64 (n, 3) -- (n, 2) on curves -- barycentric coordinates in the order of the face's corners, or of the vertices given. Returns (ids, bary) on the other side, in its naming, -1 and NaN for a point with no image; with warnings=True, (ids, bary, warnings), what this call found where the map is not a bijection. The GIL is released while mapping: several threads may map through one object.
        """
    def to_output(self, ids: typing.Any, bary: typing.Any, warnings: bool = False) -> tuple:
        """
        Points on the input, to the output. ids: int64 (n,), a face (segment) per point, or (n, 3) -- (n, 2) on curves -- vertices where the side names points by the vertices of a mesh (output_by_vertices, input_by_vertices); -1 for none. bary: float64 (n, 3) -- (n, 2) on curves -- barycentric coordinates in the order of the face's corners, or of the vertices given. Returns (ids, bary) on the other side, in its naming, -1 and NaN for a point with no image; with warnings=True, (ids, bary, warnings), what this call found where the map is not a bijection. The GIL is released while mapping: several threads may map through one object.
        """
    @property
    def build_warnings(self) -> typing.Any:
        """
        What building the map found where it is not a bijection, by kind.
        """
    @property
    def input_by_vertices(self) -> bool:
        """
        Whether input points are named by three vertices of the tet mesh the history starts from, not by triangles of input files.
        """
    @property
    def input_faces(self) -> numpy.ndarray:
        """
        int64 (m, 3): the triangles of the input files, in order, as the history holds them.
        """
    @property
    def input_triangles(self) -> numpy.ndarray:
        """
        int64 (m, 3): the input's tracked faces, in the ids input points use.
        """
    @property
    def output_by_vertices(self) -> bool:
        """
        Whether output points are named by three vertices of a tet mesh, not by triangles.
        """
    @property
    def output_triangles(self) -> numpy.ndarray:
        """
        int64 (k, 3): the output's tracked faces, in the ids output points use.
        """
class CurveTracking:
    """
    The map between a curve application's input curves and its output's (triwild), both ways, built from the history it recorded: the bytes of report['history'], or CurveTracking.load(path). Points on the input are named by segments of the input curve files -- the segments of all of them, in order, a polyline's consecutive vertex pairs -- and points on the output by the two vertices of a tracked edge of the output mesh (output_by_vertices). Mapping never changes the object.
    """
    @staticmethod
    def load(path: os.PathLike | str | bytes, smoothing: str = 'flatten') -> CurveTracking:
        """
        Build the map from a curve history file, as from its bytes.
        """
    def __init__(self, history: bytes, smoothing: str = 'flatten') -> None:
        """
        Build the map from the bytes of a curve history. smoothing: 'flatten' or 'barycentric'.
        """
    def feature_points(self) -> numpy.ndarray:
        """
        int64 (p, 2): the input's feature points -- curve ends, junctions, input points -- each a vertex of the input files (the curve files', in order, then the point files') and the output vertex it ended at, -1 for none.
        """
    def to_input(self, ids: typing.Any, bary: typing.Any, warnings: bool = False) -> tuple:
        """
        Points on the output curves, to the input: the inverse map. The two vertices may be given in either order, the coordinates following them. ids: int64 (n,), a face (segment) per point, or (n, 3) -- (n, 2) on curves -- vertices where the side names points by the vertices of a mesh (output_by_vertices, input_by_vertices); -1 for none. bary: float64 (n, 3) -- (n, 2) on curves -- barycentric coordinates in the order of the face's corners, or of the vertices given. Returns (ids, bary) on the other side, in its naming, -1 and NaN for a point with no image; with warnings=True, (ids, bary, warnings), what this call found where the map is not a bijection. The GIL is released while mapping: several threads may map through one object.
        """
    def to_output(self, ids: typing.Any, bary: typing.Any, warnings: bool = False) -> tuple:
        """
        Points on the input curves, to the output. ids: int64 (n,), a face (segment) per point, or (n, 3) -- (n, 2) on curves -- vertices where the side names points by the vertices of a mesh (output_by_vertices, input_by_vertices); -1 for none. bary: float64 (n, 3) -- (n, 2) on curves -- barycentric coordinates in the order of the face's corners, or of the vertices given. Returns (ids, bary) on the other side, in its naming, -1 and NaN for a point with no image; with warnings=True, (ids, bary, warnings), what this call found where the map is not a bijection. The GIL is released while mapping: several threads may map through one object.
        """
    @property
    def build_warnings(self) -> typing.Any:
        """
        What building the map found where it is not a bijection, by kind.
        """
    @property
    def input_segments(self) -> numpy.ndarray:
        """
        int64 (m, 2): the segments of the input curve files, in order.
        """
    @property
    def output_by_vertices(self) -> bool:
        """
        Whether output points are named by two vertices of a mesh, not by segments.
        """
    @property
    def output_segments(self) -> numpy.ndarray:
        """
        int64 (k, 2): the output's tracked edges, in the ids output points use.
        """
class VolumeTracking:
    """
    The map of a tet mesh's volume, region by region, between the mesh its volume history starts from and the one it ends at (tetwild or simwild with volume_history_output), both ways: built from the surface history and the volume history the application recorded -- the bytes a memory call returns as report['history'] and report['volume_history'], or VolumeTracking.load() of the files -- and the two meshes, which name points by their tets: the mesh the histories start from (history_input) and the one they end at (simwild's mesh, tetwild's history_output, the mesh before its filter). A point keeps its region, the tets with the same tags. Without the meshes it maps positions only (map_positions). Mapping releases the GIL and changes only the operation maps it keeps (cache_bytes), under a lock: threads may share one object.
    """
    @staticmethod
    def load(surface_history: os.PathLike | str | bytes, volume_history: os.PathLike | str | bytes, input_mesh: os.PathLike | str | bytes, output_mesh: os.PathLike | str | bytes, cache_bytes: typing.SupportsInt | typing.SupportsIndex = 268435456) -> VolumeTracking:
        """
        Build the map from the files the file route writes: the two histories and the two meshes (<output>_history_input.msh, and simwild's <output>.msh or tetwild's <output>_history_output.msh).
        """
    @typing.overload
    def __init__(self, surface_history: bytes, volume_history: bytes, input_mesh: typing.Any, output_mesh: typing.Any, cache_bytes: typing.SupportsInt | typing.SupportsIndex = 268435456) -> None:
        """
        Build the map from the bytes of both histories and the two meshes, as wm.read_mesh and the memory calls return them. cache_bytes: about how many bytes of operation maps to keep for later calls, 0 for none.
        """
    @typing.overload
    def __init__(self, surface_history: bytes, volume_history: bytes, cache_bytes: typing.SupportsInt | typing.SupportsIndex = 268435456) -> None:
        """
        Build the map from the bytes of both histories alone, without the meshes: it maps positions (map_positions) only.
        """
    def map_positions(self, xyz: typing.Any, direction: str = 'to_output', regions: typing.Any = None) -> tuple:
        """
        Positions where the volume history starts to where it ends (direction 'to_output'), or back ('to_input'). xyz: float64 (n, 3). regions: int64 (n,), each position's region (an index into regions) as a call the other way returned it, -1 for the region of the tet an operation first finds it in -- the default for all. Returns (xyz, region): where they end and the region each ends in, -1 for a position no operation took. A position that is not a number comes back as it is.
        """
    def to_input(self, tets: typing.Any, bary: typing.Any) -> tuple:
        """
        Points of the output mesh to the input mesh, the inverse map; as to_output.
        """
    def to_output(self, tets: typing.Any, bary: typing.Any) -> tuple:
        """
        Points of the input mesh to the output mesh. tets: int64 (n,), a row of the input mesh's tets per point, -1 for none; bary: float64 (n, 4), barycentric coordinates in the order of its corners. Returns (tets, bary) in the output mesh, a tet of the point's own region, -1 and NaN for a point with no tet there (outside the mesh, or no point: no tet, coordinates that are not numbers).
        """
    @property
    def has_meshes(self) -> bool:
        """
        Whether it has the meshes, which to_output and to_input name points by.
        """
    @property
    def regions(self) -> list[list[int]]:
        """
        The regions, each the tags of its tets, sorted: what a region index names.
        """
    @property
    def stats(self) -> typing.Any:
        """
        What building and using the map found so far: the operation maps built, failed (with the reasons), kept, and the points that changed region or went through an operation with no map.
        """
    @property
    def tag_names(self) -> dict[int, str]:
        """
        The labels the application's mesh files give the tags, by tag (simwild's; none for tetwild's one region).
        """
@typing.overload
def c1_simplification(options: dict, meshes: dict) -> tuple:
    """
    Work in progress. Simplifies a surface together with its tet mesh and UV layout, within a deviation threshold, for C1 (Clough-Tocher) surface constructions.
    
    Options (every one, with its full description: https://wildmeshing.github.io/wildmeshing-toolkit/reference/c1_simplification/):
      application (string, required): Application name must be c1_simplification.
      surface_mesh (string, required): surface mesh file name
      tet_mesh (string, required): tet mesh file name
      uv_mesh (string, required): uv mesh file name
      dofs (string, required): dofs file name
      cones (string, required): cone vids file name
      output (string, default "out"): Output file name (without extension).
      log_level (int, default 2): wmtk logger level.
      threshold (float, default 0.1): threshold of deviation
    
    With meshes: run on named NumPy mesh dictionaries and return (meshes, report) without file I/O. Component calls run one at a time per process.
    """
@typing.overload
def c1_simplification(options: dict) -> None:
    """
    Without meshes: run on files, as wmtk_app does with the same options.
    """
def components() -> list[str]:
    """
    The names of the components, each a function of this module.
    """
@typing.overload
def isotropic_remeshing(options: dict, meshes: dict) -> tuple:
    """
    Remeshes a triangle mesh towards a uniform target edge length with splits, collapses, swaps and tangential smoothing (Botsch and Kobbelt 2004). With a positive eps_rel, the surface stays inside an envelope around the input; features and feature_fields name curves and vertices the remeshing keeps.
    
    Options (every one, with its full description: https://wildmeshing.github.io/wildmeshing-toolkit/reference/isotropic_remeshing/):
      application (string, required): Application name must be isotropic_remeshing.
      input (string, required): Triangular input mesh.
      output (string, default "out.obj"): Output file name.
      input_dir (string, default ""): Directory where the input files are located.
      use_sample_envelope (bool, default false): Use sample envelope instead of exact one.
      num_threads (int, default 0): Number of threads.
      max_iterations (int, default 3): Number of iterations, each a pass of splits, collapses, swaps and smoothing.
      eps_rel (float, default -1): Envelope thickness relative to the bounding box diagonal.
      boundary_eps_rel (float, default -1): Thickness of a tube around the open boundary of the input, relative to the bounding box diagonal.
      length_rel (float, default -1): Target edge length relative to the bounding box.
      length_abs (float, default -1): Absolute target edge length.
      freeze_boundary (bool, default true): Freeze the vertices of the input's open boundary.
      log_file (string, default ""): Logs are not just printed on the terminal but also saved in this file.
      feature_fields (list, default []): Triangle fields of a .msh input, as io::read_mesh reads it, whose boundaries the remeshing keeps as curves (see IsotropicRemeshing::set_feature_edges): per-face fields (labels), where two faces differ, and per-corner fields (uv and normal, known by their names: two and three values per corner, corner after corner, so 6 and 9 columns), where the two faces at an edge disagree at either end -- a seam.
      features (string, default ""): File route: a JSON file, its path resolved like input, of curves and vertices to keep: {"edges": [[a, b], ...], "edge_ids": [id, ...], "vertices": [[v, id], ...]}, every key optional.
      history_output (string, default ""): Record every committed operation and save the history here (see wmtk/history): the input surface and its edits, from which the map between input and output is rebuilt (the tracking application).
      report (string, default ""): A JSON file that stores information about the result and the method execution, e.g., runtime.
      use_link_condition (bool, default true): Require the link condition before collapsing an edge.
      check_inversions (bool, default true): Refuse a collapse, swap or smoothing that folds the surface over: one that leaves more triangles facing against, or flat to, the surface it replaces (the triangles it changes or removes, their area vectors summed) or the surface it leaves than there were, or more such triangles in the stars of the vertices it touches, each star against its own normal.
      swap_quality (bool, default false): Swap an edge where that raises the worse shape quality (4 sqrt(3) area / sum of squared edge lengths) of its two triangles, as the remeshing app of the topology-first B-rep pipeline does, instead of where it brings the valences of its four vertices closer to 6 (4 on the boundary).
      limit_collapse_length (bool, default false): Refuse a collapse that leaves an edge longer than 4/3 of the target length, the length above which an edge is split, as Botsch and Kobbelt's isotropic remeshing does.
      cleanup_angle (float, default 0): Repair, at the end of every iteration, the triangles whose smallest angle is under this many degrees; 0 repairs none.
      remove_duplicate_eps (float, default 1e-05): Merge input vertices closer than this fraction of the bounding box diagonal.
      preallocation_factor (float, default 6.0): Mesh storage (connectivity + attributes) is preallocated to this factor times the live element count at init and consolidation.
    
    With meshes: run on named NumPy mesh dictionaries and return (meshes, report) without file I/O. Component calls run one at a time per process.
    """
@typing.overload
def isotropic_remeshing(options: dict) -> None:
    """
    Without meshes: run on files, as wmtk_app does with the same options.
    """
@typing.overload
def manifold_extraction(options: dict, meshes: dict) -> tuple:
    """
    Makes the boundary of a tagged selection of a triangle (2D) or tet (3D) mesh manifold: around each of its non-manifold vertices and edges it carves an offset of the given radius and fills it with fill_tags (ambient by default).
    
    Options (every one, with its full description: https://wildmeshing.github.io/wildmeshing-toolkit/reference/manifold_extraction/):
      application (string, required): Application name must be manifold_extraction.
      input (string, required): Input mesh, .msh: a tagged triangle (2D) or tet (3D) mesh.
      output (string, default "out"): Output file name (without extension).
      input_dir (string, default ""): Directory where the input files are located.
      tag_selection (string, default "tag_0"): Tag selection, the boundary of which is taken to be the input surface to make manifold.
      fill_tags (list, default []): Tag set to fill non-manifold offsets with.
      radius_rel (float, default 0.01): Radius to offset around non manifold components, relative to bounding box diagonal.
      radius (float, default -1.0): Radius to offset around non manifold components.
      DEBUG_output (bool, default false): Write the tet mesh as out_{}.vtu after every operation.
      save_vtu (bool, default false): Save .vtu of output mesh
      write_surface (bool, default false): Write the output surface (OBJ) after manifold extraction.
      log_file (string, default ""): Logs are not just printed on the terminal but also saved in this file.
      report (string, default ""): A JSON file that stores information about the result and the method execution, e.g., runtime.
      envelope (string, default ""): Memory route only: the name of the input mesh that holds the envelope surface of a tagged mesh input -- the `envelope` mesh simwild returns with its result.
    
    With meshes: run on named NumPy mesh dictionaries and return (meshes, report) without file I/O. Component calls run one at a time per process.
    """
@typing.overload
def manifold_extraction(options: dict) -> None:
    """
    Without meshes: run on files, as wmtk_app does with the same options.
    """
@typing.overload
def prismatic_mesh(options: dict, meshes: dict) -> tuple:
    """
    Work in progress. From a topological_offset construction (a .vtu, or a .msh with corr_input_vid), thins the offset band around the solid to a target thickness and, with prism_dominant, rebuilds it as prisms, pyramids and tets.
    
    Options (every one, with its full description: https://wildmeshing.github.io/wildmeshing-toolkit/reference/prismatic_mesh/):
      application (string, required): Application name must be prismatic_mesh.
      input (string, required): Construction .vtu or Gmsh 4.1 .msh with corr_input_vid.
      output (string, required): Output .vtu or .msh (Gmsh 4.1 ASCII) filename, relative to the JSON directory.
      input_dir (string, default ""): Base directory for input; set by wmtk_app to the JSON directory.
      thicknessratio (float, default 0.1): Positive target thickness relative to the mean length of unique input surface edges.
      iterations (int, default 5): Number of collapse/smoothing iterations; 0 computes targets without optimization.
      keep_background_mesh (bool, default false): Keep exterior background tetrahedra during optimization/output.
      prism_dominant (bool, default false): After optimization, reconstruct conforming prisms, input-apex shell pyramids and tetrahedra.
      export_input_and_shell (bool, default false): Additionally export the union of input volume and offset band as <output_stem>_input_shell.msh and .vtu, using the same final coordinates, hybrid cells and attributes while omitting background cells.
      min_tet_volume (float, default 0): Optional nonnegative signed-volume floor for new or modified shell tetrahedra, candidate pyramid decompositions and new background pyramid cavity cones.
      smoothing_max_backtracks (int, default 40): Maximum smoothing step halvings after the full target step; between 0 and 60.
      jacobian_smoothing_iterations (int, default 0): Optional prism Jacobian smoothing rounds after tet optimization.
      jacobian_target (float, default 0.01): Positive target det(J), normalized by twice input triangle area times initial mean column length.
      jacobian_position_weight (float, default 0.001): Positive quadratic weight on displacement from the position before Jacobian smoothing, normalized by target thickness (initial column length if unavailable).
      jacobian_max_step_ratio (float, default 0.25): Positive per-move displacement bound relative to thickness.
      jacobian_max_displacement_ratio (float, default 0.5): Positive total displacement bound from the position before Jacobian smoothing, relative to thickness.
      background_remeshing (bool, default false): Enable adaptive background 2-3/3-2 swaps and interior relaxation after each tet iteration, then retry collapse.
      background_remeshing_passes (int, default 2): Local remeshing passes per tet iteration; positive.
      background_quality_threshold (float, default 0.1): Trigger on background mean ratio below this threshold in (0,1]; 1 is a regular tet, 0 is degenerate.
      background_remeshing_max_operations (int, default 500): Maximum accepted background swaps/moves per tet iteration; positive.
      background_remeshing_max_attempts (int, default 20000): Maximum attempted background swaps/moves per tet iteration; positive.
    
    With meshes: run on named NumPy mesh dictionaries and return (meshes, report) without file I/O. Component calls run one at a time per process.
    """
@typing.overload
def prismatic_mesh(options: dict) -> None:
    """
    Without meshes: run on files, as wmtk_app does with the same options.
    """
@typing.overload
def qslim(options: dict, meshes: dict) -> tuple:
    """
    Simplifies a triangle mesh by collapsing the edges of least quadric error first (Garland and Heckbert 1997), each to the quadric's optimum, until the vertex count reaches the target. With a positive eps_rel, every collapse keeps the surface inside an envelope around the input.
    
    Options (every one, with its full description: https://wildmeshing.github.io/wildmeshing-toolkit/reference/qslim/):
      application (string, required): Application name must be qslim.
      input (string, required): Triangular input mesh.
      output (string, default "out.obj"): Output file name.
      input_dir (string, default ""): Directory where the input files are located.
      use_sample_envelope (bool, default false): Use sample envelope instead of exact one.
      num_threads (int, default 0): Number of threads.
      eps_rel (float, default -1): Envelope thickness relative to the bounding box diagonal.
      boundary_eps_rel (float, default -1): Thickness of a tube around the open boundary of the input, relative to the bounding box diagonal.
      target_rel (float, default 0.1): Target number of vertices relative to the number of input vertices.
      target_abs (float, default -1): Absolute target number of vertices.
      log_file (string, default ""): Logs are not just printed on the terminal but also saved in this file.
      report (string, default ""): A JSON file that stores information about the result and the method execution, e.g., runtime.
      throw_on_fail (bool, default false): Throw exception if the output does not fulfil the desired criteria.
      history_output (string, default ""): Record every committed operation and save the history here (see wmtk/history): the input surface and its edits, from which the map between input and output is rebuilt (the tracking application).
      preallocation_factor (float, default 6.0): Mesh storage (connectivity + attributes) is preallocated to this factor times the live element count at init and consolidation.
    
    With meshes: run on named NumPy mesh dictionaries and return (meshes, report) without file I/O. Component calls run one at a time per process.
    """
@typing.overload
def qslim(options: dict) -> None:
    """
    Without meshes: run on files, as wmtk_app does with the same options.
    """
def read_history(path: os.PathLike | str | bytes) -> bytes:
    """
    Read a history file (a component's history_output, of a surface or of curves, or its volume_history_output) as the bytes a memory call returns in its report's 'history' ('volume_history') and takes as 'history' ('volume_history').
    """
def read_mesh(path: os.PathLike | str | bytes) -> dict:
    """
    Read an existing supported mesh format into dictionaries of owning NumPy arrays.
    """
@typing.overload
def shortest_edge_collapse(options: dict, meshes: dict) -> tuple:
    """
    Simplifies a triangle mesh by collapsing its shortest edges first (Hoppe 1996, Progressive Meshes), until the vertex count reaches the target. With a positive eps_rel, every collapse keeps the surface inside an envelope around the input.
    
    Options (every one, with its full description: https://wildmeshing.github.io/wildmeshing-toolkit/reference/shortest_edge_collapse/):
      application (string, required): Application name must be shortest_edge_collapse.
      input (string, required): Triangular input mesh.
      output (string, default "out.obj"): Output file name (with extension).
      input_dir (string, default ""): Directory where the input files are located.
      num_threads (int, default 0): Number of threads.
      target_rel (float, default 0.1): Target number of vertices relative to input number.
      eps_rel (float, default -1): Envelope thickness relative to the bounding box diagonal.
      boundary_eps_rel (float, default -1): Thickness of a tube around the open boundary of the input, relative to the bounding box diagonal.
      use_sample_envelope (bool, default false): Use sample envelope instead of exact one.
      throw_on_fail (bool, default false): Throw exception if the output does not fulfil the desired criteria.
      use_link_condition (bool, default true): Require the link condition before collapsing an edge.
      remove_duplicate_eps (float, default 1e-05): Merge input vertices closer than this fraction of the bounding box diagonal.
      history_output (string, default ""): Record every committed operation and save the history here (see wmtk/history): the input surface and its edits, from which the map between input and output is rebuilt (the tracking application).
      preallocation_factor (float, default 6.0): Mesh storage (connectivity + attributes) is preallocated to this factor times the live element count at init and consolidation.
    
    With meshes: run on named NumPy mesh dictionaries and return (meshes, report) without file I/O. Component calls run one at a time per process.
    """
@typing.overload
def shortest_edge_collapse(options: dict) -> None:
    """
    Without meshes: run on files, as wmtk_app does with the same options.
    """
@typing.overload
def simwild(options: dict, meshes: dict) -> tuple:
    """
    Builds and edits multi-region meshes. With surfaces (3D) or curve networks (2D) as input, it meshes them all into one tet or triangle mesh with a region per input, overlaps included, and optimizes it like tetwild and triwild. With a tagged .msh as input, operation selects an edit: remeshing improves the mesh; fill_holes_topo, tight_seal_topo, keep_lcc, resolve_overlaps, replace_tags and tag_priority edit its regions.
    
    Options (every one, with its full description: https://wildmeshing.github.io/wildmeshing-toolkit/reference/simwild/):
      application (string, required): Application name must be simwild.
      input (list, required): List of input files.
      dimension (string, default "auto"): Whether the input is 2D or 3D, as a string.
      input_names (list, default []): List of names for the input meshes.
      input_type (list, default []): One entry per input, saying what its tag marks.
      operation (string, default "remeshing"): What to do.
      output (string, default "out"): Output file name (without extension).
      input_transform (list, default []): Transformation matrices for the inputs.
      input_dir (string, default ""): Directory where the input files are located.
      input_edges (list, default []): List of input feature EDGE meshes, 3D surface inputs only (a .msh input, a tagged mesh and a 2D input refuse them): curve networks forced into the tetrahedralization exactly -- every input edge is represented by a chain of tet edges tiling it, endpoint to endpoint.
      input_points (list, default []): List of input feature POINT files, 3D surface inputs and 2D curve inputs (a .msh input and a tagged mesh refuse them): every vertex of each file is forced into the mesh as a vertex, exactly -- a tet vertex in 3D, a vertex of the curve arrangement in 2D, where only x and y are used (edges in these files are ignored, with a warning).
      skip_simplify (bool, default false): If true, input simplification will be skipped.
      tag_from_winding_number (bool, default true): How the cells of the arrangement are tagged.
      split_connected_components (bool, default false): Treat each connected component of an input file as an input in its own right, rather than the file as a whole.
      use_sample_envelope (bool, default false): Use the sampled envelope instead of the exact one.
      simplify_use_sample_envelope (bool, default false): Use the sampled envelope instead of the exact one for the input simplification only -- in 3D, of the input surface before insertion without preserve_topology, of the tet mesh after insertion with it (operation remeshing); in 2D, of the input curves before insertion without preserve_topology; in every case unless skip_simplify; the optimization keeps whatever use_sample_envelope selects.
      num_threads (int, default 0): Number of threads.
      max_iterations (int, default 80): Maximum iterations before stopping.
      eps_rel (float, default 0.002): Envelope thickness relative to the bounding box diagonal.
      eps (float, default -1): Absolute envelope thickness.
      eps_simplify_rel (float, default 0.0002): Envelope thickness relative to the bounding box for the initial simplification.
      optimize_envelope_around_simplified (bool, default false): Build the optimizer's envelope around the SIMPLIFIED geometry at the remaining tolerance (eps - simplify_eps) instead of around the original input at the full eps.
      eps_simplify (float, default -1): Absolute envelope thickness for the initial simplification.
      simplify_boundary_envelope (bool, default true): Let the input simplification move the open boundary of the surface, as long as every boundary edge stays within order2_envelope_ratio * eps_simplify of the input's boundary edges (the same scaling tetwild uses).
      remove_duplicate_eps (float, default 0): Remove duplicate vertices in the input meshes that are closer than this fraction of the bounding box diagonal.
      order2_envelope_ratio (float, default 1.0): Thickness of the tube that holds the order-2 edges -- the open boundaries of the surface and its non-manifold edges -- as a fraction of the surface envelope's eps, and the radius of the anchors where they end or three or more meet.
      feature_envelope_ratio (float, default 1.0): Thickness of the feature-curve tube -- the region a tet edge tagged as tiling an input feature edge (/input_edges) must stay inside -- and the radius of the feature anchors (the /input_points, and the vertices where feature curves end or three or more meet), as a fraction of the surface envelope's eps.
      length (float, default -1): Absolute target edge length.
      length_rel (float, default 0.05): Target edge length relative to the bounding box diagonal.
      sizing_field (list, default []): Prescribe a length for a specific region defined by tags.
      quality_field (list, default []): Prescribe a target quality for a specific region defined by tags.
      stop_energy (float, default 10): Target energy.
      stop_at_float (bool, default false): Stop when all vertices are in floating point precision, i.e., no more rational numbers are needed.
      preserve_topology (bool, default true): Preserve topology of input.
      project_line_search_steps (int, default 12): Bisections tried by the projected line search before it gives up on a vertex: the step along the Newton direction is halved this many times, each candidate projected onto the input, and the first that does not invert and lowers the worst incident element is taken.
      project_line_search_nested_steps (int, default 0): After the projected line search gives up, revisit each candidate and bisect between the interpolated point and its projection this many times, taking the longest step toward the input that still does not invert and still lowers the worst element.
      smoothing_mode (string, default "projected"): How smoothing places a surface vertex.
      w_amips (float, default 0.0001): Relative weight of the AMIPS quality term against the envelope term during smoothing; the envelope weight is 1 - w_amips.
      num_smoothing_passes (int, default 2): Smoothing passes per optimization iteration when interleaved_smoothing is disabled.
      interleaved_smoothing (bool, default true): Run smoothing after each topology phase, matching TetWild and TriWild, instead of batching it after all topology operations.
      interleaved_smoothing_passes (int, default 1): Smoothing passes after each topology phase when interleaved_smoothing is enabled.
      check_surface_folds (bool, default false): Refuse a collapse, swap or smoothing that folds the tracked surface over: one that leaves more tracked faces facing against, or flat to, the surface it replaces (the tracked faces around it, their area vectors summed) or the surface it leaves than there were, or more such faces in the stars of the surface vertices it touches.
      colored_smoothing (bool, default true): Parallel smoothing (num_threads > 0) by color class: the vertices are split into classes of pairwise non-adjacent vertices, smoothed one class after another, each class fully in parallel without locks.
      write_vtu (bool, default false): Write not just MSH but also VTU output.
      log_file (string, default ""): Logs are not just printed on the terminal but also saved in this file.
      report (string, default ""): A JSON file that stores information about the result and the method execution, e.g., runtime.
      history_output (string, default ""): 3D only: a 2D run refuses it.
      volume_history_output (string, default ""): 3D only, with history_output.
      DEBUG_output (bool, default false): Write the mesh as debug_{}.vtu after every operation.
      DEBUG_sanity_checks (bool, default false): Perform sanity checks after every operation.
      allow_surface_swap (bool, default true): Allow surface edge swapping.
      check_surface_topology (bool, default false): Check if the surface topology is preserved after each operation.
      verbose_quality_stats (bool, default true): Log the per-tag quality breakdown after every optimization iteration: for each entry of /quality_field, and for the cells no entry matches, the number of cells, their max and average absolute quality, the target applied to them, and the max quality relative to that target.
      stuck_refine_stall_eps (float, default 0.1): Refine when the last iteration's improvement was at most this fraction of the distance the max energy still has to cover: (prev_max - max) <= eps * (max - stop_energy).
      stuck_refine_cooldown (int, default 1): After a refinement, skip this many improvement iterations before refining again (let the operations act on the new sizing field).
      stuck_refine_num_worst (int, default 0): Number of worst tets whose neighborhoods are refined when stuck.
      stuck_refine_rings (int, default 0): Graph rings around each worst tet included in the stuck refinement.
      stuck_refine_factor (float, default 0.5): Multiplicative reduction of the sizing scalar per stuck refinement (0.5 => halve).
      stuck_refine_min_scalar (float, default 0.001): Lower bound on the sizing scalar during stuck refinement.
      stuck_refine_gradation (float, default 2.0): Gradation cap for the monotone sizing smoothing: adjacent sizings may differ by at most this factor (smoothing only lowers values).
      stuck_refine_force_split (bool, default true): When the max energy stalls, split each worst tet's longest edge once, bypassing the split length gate, to unstick a sliver without changing the sizing field.
      skip_good_regions (bool, default false): Only smooth vertices incident to a tet whose energy is at least skip_good_regions_margin * stop_energy; skip smoothing already-good regions.
      skip_good_regions_margin (float, default 0.9): A tet is 'active' (worth smoothing around) when its energy is at least this fraction of stop_energy.
      fill_holes_tags (list, default []): For fill_holes_topo: list of tag values used to fill enclosed connected components (processed in order).
      fill_holes_threshold (float, default -1): For fill_holes_topo: only fill a connected component if its size (area in 2D, volume in 3D, in model units) is less than this threshold.
      tight_seal_tag_sets (list, default []): For tight_seal_topo: list of tag sets.
      tight_seal_threshold (float, default -1): For tight_seal_topo: only fill a hole cluster if its total size (area in 2D, volume in 3D, in model units) is less than this threshold.
      keep_lcc_tags (list, default []): For keep_lcc: list of tag values for which only the largest connected component is kept.
      keep_lcc_num (int, default 1): Number of largest connected components that are kept.
      resolve_overlaps_tags (list, default []): Resolve intersections between tags by filling the areas with intersections with the nearest of the intersecting tags.
      replace_tags_in (list, default []): Replace the tags by the output tags.
      replace_tags_out (list, default []): The output tags (written as intersection) that should replace the input tags.
      tag_priority (list, default []): List of tags for the tag_priority operation.
      coarsen_pass (bool, default false): Run a final coarsening pass after the optimization has converged.
      coarsen_local_smoothing_passes (int, default 0): Smoothing sweeps over the ring, inside each candidate collapse, before judging it.
      coarsen_smooth_ring (int, default 1): Radius smoothed inside the collapse, and the radius the accept test measures over.
      coarsen_global_smoothing_passes (int, default 1): Ordinary whole-mesh smoothing passes run between coarsening rounds.
      coarsen_max_rounds (int, default 2): Cap on the coarsen/smooth alternation; the pass also stops as soon as a round accepts no collapse.
      coarsen_max_inner_passes (int, default 1): Cap on the collapse pass's own dirty-epoch retry loop within one coarsening round; 0 is uncapped.
      coarsen_unbounded (bool, default true): Coarsen as far as the quality guarantee allows, instead of stopping at the target edge length.
      envelope (string, default ""): Memory route only: the name of the input mesh that holds the envelope surface of a tagged mesh input -- the `envelope` mesh simwild returns with its result.
    
    With meshes: run on named NumPy mesh dictionaries and return (meshes, report) without file I/O. Component calls run one at a time per process.
    """
@typing.overload
def simwild(options: dict) -> None:
    """
    Without meshes: run on files, as wmtk_app does with the same options.
    """
def spec(name: str) -> typing.Any:
    """
    A component's JSON spec: its option rules, as the reference documentation lists them.
    """
@typing.overload
def tetwild(options: dict, meshes: dict) -> tuple:
    """
    Tetrahedralizes triangle surfaces as they come -- open, self-intersecting, non-manifold -- following TetWild: the input is simplified within the envelope, embedded exactly into a Delaunay tet mesh of its bounding box, and the tet mesh is then optimized for quality while the embedded surface stays inside an envelope around the input. filter removes the tets outside. The output keeps the embedded surface as tracked faces, and the inputs' winding numbers on the tets.
    
    Options (every one, with its full description: https://wildmeshing.github.io/wildmeshing-toolkit/reference/tetwild/):
      application (string, required): Application name must be tetwild.
      input (list, required): List of triangular input meshes.
      input_edges (list, default []): List of input feature EDGE meshes: curve networks forced into the tetrahedralization exactly -- every input edge is represented by a chain of tet edges tiling it, endpoint to endpoint.
      input_points (list, default []): List of input feature POINT files: every vertex of each file is forced into the tetrahedralization as a tet vertex, exactly (edges in these files are ignored, with a warning).
      DEBUG_feature_retention (bool, default false): Sanity check: count how many feature anchors still have a mesh vertex within the anchor ball, measured BEFORE any /filter discards regions (a feature preserved through the optimization is never reported lost because extraction dropped its region; the feature outputs keep it either way), and report the worst miss in multiples of the ball radius.
      feature_envelope_ratio (float, default 1.0): Thickness of the feature-curve tube -- the region a tet edge tagged as tiling an input feature edge must stay inside -- and the radius of the feature-point anchors, as a fraction of the surface envelope's eps.
      output (string, default "out"): Output file name (without extension).
      input_names (list, default []): Names for the inputs, used for each input's winding-number field: wn_<name> in the VTU output and on the memory route.
      input_dir (string, default ""): Directory where the input files are located.
      write_vtu (bool, default true): Write the VTU output next to the MSH.
      skip_simplify (bool, default false): If true, input simplification will be skipped.
      simplify_use_sample_envelope (bool, default false): Use the sampled envelope instead of the exact one for the input simplification only; the tetrahedralisation keeps whatever use_sample_envelope selects.
      simplify_envelope_ratio (float, default 0.5): Envelope thickness used for the input simplification, as a fraction of the one used for the tetrahedralisation.
      simplify_boundary_envelope (bool, default true): Let the input simplification move the open boundary of the surface, as long as every boundary edge stays within order2_envelope_ratio * simplify_envelope_ratio * eps of the input's boundary edges.
      optimize_envelope_around_simplified (bool, default false): EXPERIMENT.
      order2_envelope_ratio (float, default 1.0): Thickness of the tube that holds the order-2 edges -- the open boundaries of the surface -- as a fraction of the surface envelope's eps, and the radius of the anchors where they end or three or more meet (see feature_envelope_ratio).
      use_sample_envelope (bool, default false): Use sample envelope instead of exact one.
      use_legacy_code (bool, default false): Use the original TetWild code for the mesh improvement.
      num_threads (int, default 0): Number of threads.
      max_iterations (int, default 80): Maximum iterations before stopping.
      filter (string, default "none"): Remove the outside region based on different criteria.
      skip_winding_number (bool, default false): Skip the finalize-phase winding numbers of the input: the whole input's (winding_number_input) and each input's (wn_<i>).
      compute_tracked_winding_number (bool, default false): Compute the winding number with respect to the tracked surface and write it (winding_number_tracked) even when filter is not 'tracked', which computes it anyway.
      compute_flood_fill (bool, default false): Compute the flood-fill parts -- the regions the tracked surface separates -- and write them (part) even when filter is not 'flood', which computes them anyway.
      eps_rel (float, default 0.001): Envelope thickness relative to the bounding box diagonal.
      length_rel (float, default 0.05): Target edge length relative to the bounding box diagonal.
      stop_energy (float, default 100): Target energy.
      split_high_valence_threshold (int, default 0): Incident-tet count above which a vertex accepts only one valence-increasing split per split pass, or 0 to disable.
      project_line_search_steps (int, default 12): Bisections tried by the projected line search before it gives up on a vertex: the step along the Newton direction is halved this many times, each candidate projected onto the input, and the first that does not invert and lowers the worst incident element is taken.
      project_line_search_nested_steps (int, default 0): After the projected line search gives up, revisit each candidate and bisect between the interpolated point and its projection this many times, taking the longest step toward the input that still does not invert and still lowers the worst element.
      smoothing_mode (string, default "projected"): How smoothing places a surface vertex.
      w_amips (float, default 0.0001): Relative weight of the AMIPS quality term against the envelope term during smoothing; the envelope weight is 1 - w_amips.
      num_smoothing_passes (int, default 2): Number of smoothing passes per optimization iteration.
      interleaved_smoothing (bool, default true): Interleave smoothing between the topology passes: split + smoothing, collapse + smoothing, swaps + smoothing, instead of split, collapse, swaps followed by num_smoothing_passes smoothing passes.
      interleaved_smoothing_passes (int, default 1): Smoothing passes to run after each topology pass when interleaved_smoothing is set.
      check_surface_folds (bool, default false): Refuse a collapse, swap or smoothing that folds the tracked surface over: one that leaves more tracked faces facing against, or flat to, the surface it replaces (the tracked faces around it, their area vectors summed) or the surface it leaves than there were, or more such faces in the stars of the surface vertices it touches.
      colored_smoothing (bool, default true): Parallel smoothing (num_threads > 0) by color class: the vertices are split into classes of pairwise non-adjacent vertices, smoothed one class after another, each class fully in parallel without locks.
      preserve_topology (bool, default false): Preserve the topology of the input surface.
      remove_duplicate_eps (float, default 0): Remove duplicate vertices in the input meshes that are closer than this fraction of the bounding box diagonal.
      allow_surface_swap (bool, default true): Allow the edge swaps (3-2, 4-4, 5-6) to operate on surface edges, as a topology-preserving surface diagonal flip.
      check_surface_topology (bool, default false): Check if the surface topology is preserved after each operation.
      throw_on_fail (bool, default false): Throw exception if the output does not fulfil the desired criteria.
      log_file (string, default ""): Logs are not just printed on the terminal but also saved in this file.
      report (string, default ""): A JSON file that stores information about the result and the method execution, e.g., runtime.
      history_output (string, default ""): Record every change to the tracked surface, from the input as inserted to the output, and save the history here (see wmtk/history): the tracking component rebuilds the map between the two from it.
      volume_history_output (string, default ""): With history_output.
      DEBUG_output (bool, default false): Write the mesh as debug_{}.vtu after every operation.
      DEBUG_sanity_checks (bool, default false): Perform sanity checks after every operation.
      DEBUG_hausdorff (bool, default false): Sanity Check: Compute and report the Hausdorff distance of the output to the input.
      DEBUG_euler (bool, default false): Sanity Check: Compute and report the Euler characteristic of the input and output surfaces and warn if they differ.
      DEBUG_disable_envelope (bool, default false): Diagnostic only.
      max_expected_iterations (int, default 0): Fail the run if mesh_improvement needed more than this many iterations, even if it did reach stop_energy.
      stuck_refine_stall_eps (float, default 0.1): Refine when the last iteration's improvement was at most this fraction of the distance the max energy still has to cover: (prev_max - max) <= eps * (max - stop_energy).
      stuck_refine_cooldown (int, default 1): After a refinement, skip this many improvement iterations before refining again (let the operations act on the new sizing field).
      stuck_refine_num_worst (int, default 0): Number of worst tets whose neighborhoods are refined when stuck.
      stuck_refine_rings (int, default 0): Graph rings around each worst tet included in the stuck refinement.
      stuck_refine_factor (float, default 0.5): Multiplicative reduction of the sizing scalar per stuck refinement (0.5 => halve).
      stuck_refine_min_scalar (float, default 0.001): Lower bound on the sizing scalar during stuck refinement.
      stuck_refine_gradation (float, default 2.0): Gradation cap for the monotone sizing smoothing: adjacent sizings may differ by at most this factor (smoothing only lowers values).
      stuck_refine_force_split (bool, default true): When the max energy stalls, split each worst tet's longest edge once, bypassing the split length gate, to unstick a sliver without changing the sizing field.
      skip_good_regions (bool, default false): Only smooth vertices incident to a tet whose energy is at least skip_good_regions_margin * stop_energy; skip smoothing already-good regions.
      skip_good_regions_margin (float, default 0.9): A tet is 'active' (worth smoothing around) when its energy is at least this fraction of stop_energy.
      coarsen_pass (bool, default false): Run a final coarsening pass after the optimization has converged.
      coarsen_local_smoothing_passes (int, default 0): Smoothing sweeps over the ring, inside each candidate collapse, before judging it.
      coarsen_smooth_ring (int, default 1): Radius smoothed inside the collapse, and the radius the accept test measures over.
      coarsen_global_smoothing_passes (int, default 1): Ordinary whole-mesh smoothing passes run between coarsening rounds.
      coarsen_max_rounds (int, default 2): Cap on the coarsen/smooth alternation; the pass also stops as soon as a round accepts no collapse.
      coarsen_max_inner_passes (int, default 1): Cap on the collapse pass's own dirty-epoch retry loop within one coarsening round; 0 is uncapped.
      coarsen_unbounded (bool, default true): Coarsen as far as the quality guarantee allows, instead of stopping at the target edge length.
    
    With meshes: run on named NumPy mesh dictionaries and return (meshes, report) without file I/O. Component calls run one at a time per process.
    """
@typing.overload
def tetwild(options: dict) -> None:
    """
    Without meshes: run on files, as wmtk_app does with the same options.
    """
@typing.overload
def topological_offset(options: dict, meshes: dict) -> tuple:
    """
    Adds a region at a target distance around a selection of a tagged tet mesh (the 2D offset, on triangle meshes, is refused for now): the boundary of a tag, an expression of tags, or an open curve or sheet. It grows the offset band from the selection and optimizes the mesh until the band's front lies at the target distance, keeping the band's topology throughout.
    
    Options (every one, with its full description: https://wildmeshing.github.io/wildmeshing-toolkit/reference/topological_offset/):
      application (string, required): Application name must be topological_offset.
      input (string, required): Input mesh, .msh: a tetrahedral mesh (a triangle mesh is refused for now: the 2D offset is being brought up to date).
      output (string, default "out"): Output file name (without extension).
      input_dir (string, default ""): Directory where the input files are located.
      offset_selection (string, default "!_"): Boolean expression for input simplicial complex to offset.
      offset_output_tags (list, default ["newtag"]): Tags to add to elements in the resulting offset region.
      protected_tags (list, default []): Set of tags that will not be overwritten by the offset.
      offset_in (bool, default false): Only relevant for single body mode.
      offset_out (bool, default true): Only relevant for single body mode.
      target_distance_rel (float, default 0.01): Target offset distance relative to the bounding box diagonal of the mesh.
      target_distance (float, default -1.0): Target distance for offset.
      throw_on_nonconvergence (bool, default false): If true, a run that finishes without meeting the convergence criteria raises an error instead of logging a warning.
      optimize_offset (bool, default true): Run the optimization (optimize_offset: pre-loop setup, the loop's turns, the final pass and the convergence verdict) after the offset is constructed.
      envelope_size_rel (float, default 0.001): Half-width, relative to the bounding box diagonal, of the envelope that holds the held surfaces during optimization; any operation that would push a held face outside it is rejected, and a held vertex is smoothed toward it.
      envelope_size (float, default -1.0): Absolute half-width of the envelope that holds the held surfaces (see envelope_size_rel).
      offset_field (string, default "euclidean"): Which field defines the offset, and so what the front is placed on, what the criterion measures and what the sizing field refines by.
      offset_dhat_factor (float, default 2.0): Support radius of the smooth offset potential, as a multiple of target_distance.
      front_conv (float, default -1.0): THE convergence epsilon, as an ABSOLUTE length in model units; <= 0 (the default -1): derived from front_conv_rel.
      front_conv_rel (float, default 0.1): THE convergence epsilon as a fraction of target_distance, used when front_conv is <= 0; ignored if front_conv > 0.
      stencil_order (int, default 1): 3D: the lattice below is EXPERIMENTAL_stencil_type "subdivision", the default; "interior" and "fem" (2026-10-27 / 28) have their own lattices for the same order k and need k >= 1 (see there).
      EXPERIMENTAL_stencil_type (string, default "subdivision"): 3D (2026-10-28, Spencer: "standard fem triangle points").
      front_measure (string, default "vertex_ring"): Which measure the loop exits on and refines by, in 3D and in 2D.
      sorted_marching (bool, default false): Execute the marching in decreasing order of edge length.
      save_offset_correspondence (bool, default false): 2D and 3D, and only with optimize_offset false (the run refuses the combination with true).
      sphere_trace_target_rel_tol (float, default 0.01): In (0, 1).
      construction_mode (string, default "max_marchable_fallback"): How the marching constructs the offset.
      init_optimize (bool, default false): Default false, as the band-volume runs ran it.
      repulsion_smoothing_passes (int, default 0): 2D and 3D.
      repulsion_rounds (int, default 0): 2D and 3D.
      EXPERIMENTAL_split_energy_gate (bool, default false): EXPERIMENTAL (2026-10-09), 3D.
      EXPERIMENTAL_band_volume_rule (string, default "exact"): EXPERIMENTAL (2026-10-12), 3D.
      settle_stall_eps (float, default 0.1): 3D, EXPERIMENTAL_band_volume_rule "surface" only (a non-default value under another rule is refused), with EXPERIMENTAL_settle_refinement.
      EXPERIMENTAL_settle_test (string, default "stall"): EXPERIMENTAL (2026-10-27), 3D, EXPERIMENTAL_band_volume_rule "surface" only ("bar" under another rule is refused), with EXPERIMENTAL_settle_refinement: which test decides that a turn has settled.
      EXPERIMENTAL_settle_mark (string, default "vertex_edge"): EXPERIMENTAL (2026-10-09), with EXPERIMENTAL_settle_refinement: which test marks a front face on a settled turn (the settle, the halving and the exit are unchanged).
      EXPERIMENTAL_gap_refinement (bool, default false): EXPERIMENTAL (2026-10-10), 3D, needs EXPERIMENTAL_settle_refinement.
      EXPERIMENTAL_damping (bool, default false): EXPERIMENTAL (2026-10-15), 3D.
      EXPERIMENTAL_damping_factor (float, default 0.5): EXPERIMENTAL (2026-10-15), 3D, with EXPERIMENTAL_damping.
      optimize_construction (bool, default true): 3D, with optimize_offset (since 2026-10-21; replaces EXPERIMENTAL_construction_tetwild_run, EXPERIMENTAL_refine_construction and EXPERIMENTAL_refine_construction_factor, which the strict spec now refuses).
      growth_mode (string, default "none"): 3D (since 2026-10-21; "linear" is the former EXPERIMENTAL_incremental_growth, which the strict spec now refuses with EXPERIMENTAL_incremental_growth_steps).
      linear_growth_steps (int, default 4): 3D, growth_mode "linear" (since 2026-10-21; was EXPERIMENTAL_incremental_growth_steps): N, the number of equal steps from the distance the march traced the front to, d_0, to target_distance.
      smart_growth_attempts (int, default 2): 3D, growth_mode "smart" (since 2026-10-21): the turns each attempt toward a larger target gets to converge (level 0, the loop at d_0, has max_rounds).
      smart_growth_max_fails (int, default 10): 3D, growth_mode "smart" (since 2026-10-21): how many attempts from one converged distance d_b may fail -- each failure halves the step toward target_distance -- before the run stops with an error.
      foldover_angle (float, default 30.0): 3D.
      DEBUG_stencil (bool, default false): DEBUG (2026-10-29), 3D, with DEBUG_output (or DEBUG_remote_output; refused otherwise).
      TESTING_allow_mult_comp (bool, default false): TESTING (2026-10-16), 3D.
      EXPERIMENTAL_settle_refinement (bool, default true): EXPERIMENTAL (2026-10-09), 3D.
      check_manifoldness (bool, default true): After performing offset, check if offset region is manifold
      save_vtu (bool, default false): Save .vtu of output mesh
      phi_grid_resolution (int, default 0): Samples per side of the grid the smooth offset potential is written on, as <output>_phi.vtu, for the viewer.
      DEBUG_output (bool, default false): Write debug VTU frames of the evolving mesh, in both dimensions: one frame after every operation pass and every smoothing pass, as sequential <output>_NNNNN.vtu files so ParaView loads them as one time series, plus <output>_frames.txt carrying one 'NNNNN<tab>r<turn><phase><pass>_<op>' line per frame, which the viewer scripts read for their labels; in 3D also <output>_energy.txt, one 'NNNNN<tab>label<tab>band term<tab>AMIPS term' line per frame, the energy's two parts (see offset_amips_weight).
      DEBUG_remote_output (bool, default false): DEBUG (2026-10-20), 3D.
      log_file (string, default ""): Logs are not just printed on the terminal but also saved in this file.
      report (string, default ""): A JSON file that stores information about the result and the method execution, e.g., runtime.
      num_threads (int, default 0): Number of threads used for parallel execution (smoothing, edge collapse).
      num_smoothing_passes (int, default 1): Smoothing passes after the ONE combined split+collapse+swap round when interleaved_smoothing is OFF (the default is OFF since 2026-10-29, ON from 2026-09-28) -- in the loop (3D since 2026-09-25, 2D since 2026-10-05), and in the shared optimization loop, which in this component runs only in the TetWild passes, 2D and 3D alike.
      length_rel (float, default 0.05): Target edge length relative to the bounding box diagonal.
      length (float, default -1.0): Target edge length (absolute).
      stop_energy (float, default 100.0): Target AMIPS quality for the elements.
      max_iterations (int, default 80): Cap on the shared TriWild/TetWild loop, which runs only in the TetWild passes: the final pass, and in 3D the construction pass (optimize_construction).
      interleaved_smoothing (bool, default false): Run split, collapse and swap as three separate rounds with `interleaved_smoothing_passes` smoothing passes after each, instead of one combined round followed by `num_smoothing_passes`.
      interleaved_smoothing_passes (int, default 2): Smoothing passes after each operation group (split, collapse, swap) when interleaved_smoothing is on -- in the shared TriWild/TetWild loop (the TetWild passes) and in the loop.
      colored_smoothing (bool, default false): Parallel smoothing (num_threads > 0) by color class: the vertices are split into classes of pairwise non-adjacent vertices, smoothed one class after another, each class fully in parallel without locks.
      pre_smooth (bool, default false): The loop only, not the final pass.
      coarsen_pass (bool, default false): Run the post-optimization coarsening pass, as TriWild and TetWild do.
      coarsen_unbounded (bool, default true): Coarsen as far as the quality guarantee allows, instead of stopping at the target edge length.
      coarsen_max_rounds (int, default 2): Cap on the coarsen/smooth alternation; the pass also stops as soon as a round accepts no collapse.
      coarsen_global_smoothing_passes (int, default 1): Ordinary whole-mesh smoothing passes run between coarsening rounds.
      coarsen_smooth_ring (int, default 1): How far the local smoothing after each coarsening collapse reaches, in rings; the pass locks one ring wider than this.
      coarsen_local_smoothing_passes (int, default 0): Smoothing sweeps over the ring inside each candidate coarsening collapse, before its max energy is measured, so the collapse is judged on a relaxed neighbourhood rather than on the configuration it lands in.
      EXPERIMENTAL_coarsen_guard (string, default "bar"): EXPERIMENTAL (2026-10-28), 3D, read only with coarsen_pass true (refused otherwise).
      stuck_refine_stall_eps (float, default 0.1): Fire the sizing refinement when an iteration's improvement is small next to the distance the metric still has to cover: (prev - cur) <= stall_eps * (cur - target).
      stuck_refine_cooldown (int, default 1): Iterations to skip after a sizing refinement before another may fire.
      stuck_refine_num_worst (int, default 0): How many worst faces seed the refinement.
      stuck_refine_rings (int, default 0): Vertex rings grown around each seeded face before the sizing scalar is lowered.
      stuck_refine_factor (float, default 0.5): Multiplicative reduction of the per-vertex sizing scalar per refinement.
      stuck_refine_min_scalar (float, default 0.001): Floor on the per-vertex sizing scalar.
      stuck_refine_gradation (float, default 2.0): Neighbouring sizing scalars may differ by at most this factor.
      stuck_refine_force_split (bool, default true): On a stall, also split each seeded face's longest edge once, bypassing the length gate and without touching the sizing field.
      sizing_collapse_min (bool, default true): What the surviving vertex of a collapse keeps as its sizing scalar.
      max_rounds (int, default 30): The outer loop's budget, in turns, one turn being split, collapse and swap with smoothing after each, followed by the front test.
      min_edge_length (float, default -1.0): l_min: the shortest edge the sizing field may ask for, in absolute units.
      min_edge_length_rel (float, default 0.1): l_min as a multiple of target_distance, used when min_edge_length is negative.
      min_sizing_scalar (float, default 0.001): Lower bound for the per-vertex sizing field (see max_sizing_scalar).
      max_sizing_scalar (float, default 1.0): Upper bound for the per-vertex sizing field, as a multiplier on the target edge length (splitting_l2/collapsing_l2).
      sizing_gradation (float, default 2.0): Gradation cap for the sizing field: neighbouring vertices' sizing scalars may differ by at most this factor.
      sizing_gradation_mode (string, default "ring"): How a lowered sizing scalar spreads to the vertices around it, at every place the offset lowers the field (the sag rule, the stuck refine).
      split_high_valence_threshold (int, default 0): Incident-cell count above which a vertex in a split edge's link accepts only one valence-increasing split per pass, or 0 to disable.
      project_line_search_steps (int, default 12): Bisections tried by the projected line search before it gives up on a vertex: the step along the Newton direction is halved this many times, each candidate projected onto the input, and the first that does not invert and lowers the worst incident element is taken.
      project_line_search_nested_steps (int, default 0): After the projected line search gives up, revisit each candidate and bisect between the interpolated point and its projection this many times, taking the longest step toward the input that still does not invert and still lowers the worst element.
      offset_amips_weight (float, default 1.0): Under EXPERIMENTAL_band_volume_rule "surface" w multiplies the AMIPS term only (the offset term is unweighted, 2026-10-28): w is the ratio of the two terms.
      EXPERIMENTAL_front_pocket_split (bool, default true): EXPERIMENTAL (2026-10-22), 3D.
      EXPERIMENTAL_front_spanned_split (bool, default true): EXPERIMENTAL (2026-10-24), 3D, needs EXPERIMENTAL_front_pocket_split.
      EXPERIMENTAL_exit_no_folds (bool, default true): EXPERIMENTAL (2026-10-26; foldover faces only since 2026-10-27), 3D.
      EXPERIMENTAL_label_cleanup (string, default "one_sided_at_target"): EXPERIMENTAL (2026-10-26), 3D, with EXPERIMENTAL_front_pocket_split true (refused otherwise).
      DEBUG_crossings (bool, default false): Log-only: after every operation pass and every smoothing pass, count the front vertices whose ring measure crossed the bar (from <= 1 to > 1, and back), plus new vertices already over it; inside the front smoother, classify each accepted move whose ring-measure crossing it caused as the moved vertex's own or a front neighbour's.
      smoothing_mode (string, default "projected"): How smoothing places a surface vertex.
      w_amips (float, default 0.0001): Relative weight of the AMIPS quality term against the envelope term during smoothing of envelope-held vertices (the engine's input-surface envelope term); the envelope weight is 1 - w_amips.
      perform_sanity_checks (bool, default false): Check after every operation pass that no element is inverted and no tracked-surface simplex left its envelope.
      use_rest_pose (bool, default false): 3D.
      envelope (string, default ""): Memory route only: the name of the input mesh that holds the envelope surface of a tagged mesh input -- the `envelope` mesh simwild returns with its result.
    
    With meshes: run on named NumPy mesh dictionaries and return (meshes, report) without file I/O. Component calls run one at a time per process.
    """
@typing.overload
def topological_offset(options: dict) -> None:
    """
    Without meshes: run on files, as wmtk_app does with the same options.
    """
@typing.overload
def tracking(options: dict, meshes: dict) -> tuple:
    """
    Rebuilds the map between an application's input surface and its output from the history the application recorded (history_output): one bijective local map per operation, composed in order. operation selects what to do with it: report on it (info), map points (map), carry fields (transfer), follow feature points (features), or do the same through a tet mesh's volume (volume, map_volume).
    
    Options (every one, with its full description: https://wildmeshing.github.io/wildmeshing-toolkit/reference/tracking/):
      application (string, required): Application name must be tracking.
      history (string, required): A history, recorded by an application's history_output, from which the map is rebuilt: a surface's (isotropic_remeshing, qslim, shortest_edge_collapse, tetwild, simwild) or curves' (triwild); the history says which.
      input (string, default ""): transfer: the surface the history starts from -- the same triangles in the same order: a gmsh file, as io::read_mesh (wm.read_mesh) reads it, whose triangle fields are carried onto the output -- per face (labels), or per corner (uv and normal, three values per corner, corner after corner: texture coordinates) -- or a triangle mesh of another format, as read, with none.
      output (string, default "out.msh"): transfer: the output surface, written as a gmsh file through io::write_mesh, the carried fields as attributes of its triangles: what wm.read_mesh reads.
      output_obj (string, default ""): transfer: also write the output as a textured Wavefront OBJ, with uv_field as its texture coordinates and a material mapping `texture`.
      uv_field (string, default "uv"): transfer: the per-corner field output_obj takes its texture coordinates from (the first two values of each corner).
      texture (string, default "checker.png"): transfer: the image the material of output_obj maps.
      input_dir (string, default ""): Directory where the input files are located.
      operation (string, default "info"): info: build the map and report what it holds, where it stops being a bijection, and how far the output's face centroids (segment midpoints, for curves) move when mapped to the input and back.
      log_file (string, default ""): Logs are not just printed on the terminal but also saved in this file.
      tet_mesh (string, default ""): transfer: the tet mesh (gmsh) the history's application wrote, whose tracked surface is where the history ends.
      output_tet_mesh (string, default ""): transfer: where to write tet_mesh with the fields on its tracked surface.
      points (string, default ""): map: a text file of points, one per line, numbers separated by blanks, '#' starting a comment.
      output_points (string, default ""): map: where to write the mapped points, one line per point of `points`, in the same order, as described for `points` on the side they are mapped to, relative to input_dir as `points` is.
      direction (string, default "to_output"): map: to_output maps points on the input to the output; to_input maps points on the output back to the input, the inverse map.
      smoothing (string, default "flatten"): How the map follows a vertex smoothing step.
      scaffold (string, default "box"): The scaffold of each local map's joint flattening, which keeps the outline the two patches share a simple polygon while SLIM lowers their distortion.
      collapse_patch (string, default "both_stars"): The faces each collapse's local map flattens.
      volume_history (string, default ""): volume, map_volume: the volume history (an application's volume_history_output) recorded with `history`, the surface history.
      samples (int, default 1000): volume: how many points to sample near the tracked surface of tet_mesh.
    
    With meshes: run on named NumPy mesh dictionaries and return (meshes, report) without file I/O. Component calls run one at a time per process.
    """
@typing.overload
def tracking(options: dict) -> None:
    """
    Without meshes: run on files, as wmtk_app does with the same options.
    """
@typing.overload
def triwild(options: dict, meshes: dict) -> tuple:
    """
    Triangulates the plane around curve networks as they come -- polylines that cross, overlap or leave gaps -- following TriWild: the curves are simplified within the envelope, embedded exactly into a triangle mesh around them, and the mesh is then optimized for quality while the embedded curves stay inside an envelope around the input. filter removes the triangles outside.
    
    Options (every one, with its full description: https://wildmeshing.github.io/wildmeshing-toolkit/reference/triwild/):
      application (string, required): Application name must be triwild.
      input (list, required): List of input curve networks: OBJ files of polylines ('v' and 'l' lines).
      input_points (list, default []): List of input POINT files: every vertex of each file is a feature point to preserve, whether or not the file also has edges (they are ignored).
      output (string, default "out"): Output file name (without extension).
      input_names (list, default []): List of names for the input meshes.
      input_dir (string, default ""): Directory where the input files are located.
      num_threads (int, default 0): Number of threads.
      max_iterations (int, default 80): Maximum iterations before stopping.
      skip_simplify (bool, default false): If true, input simplification will be skipped.
      simplify_envelope_ratio (float, default 0.5): Envelope thickness used for the input simplification, as a fraction of the one used for the triangulation.
      optimize_envelope_around_simplified (bool, default false): Build the optimizer's envelope around the SIMPLIFIED geometry at the remaining tolerance (eps - simplify_eps) instead of around the original input at the full eps.
      simplify_use_sample_envelope (bool, default false): Use the sampled envelope instead of the exact one for the input simplification only; the triangulation keeps whatever use_sample_envelope selects.
      use_sample_envelope (bool, default false): Use the sampled envelope instead of the exact one.
      filter (string, default "none"): Remove the outside region based on different criteria.
      skip_winding_number (bool, default false): Skip the finalize-phase winding numbers of the inputs.
      compute_flood_fill (bool, default false): Compute the flood-fill parts -- the regions the tracked curves separate -- and write them (the VTU field flood_fill) even when filter is not 'flood', which computes them anyway.
      eps_rel (float, default 0.001): Envelope thickness relative to the bounding box diagonal.
      remove_duplicate_eps (float, default 0): Merge vertices in the input meshes that are closer than this fraction of the bounding box diagonal, dropping the edges that become degenerate or duplicated.
      DEBUG_hausdorff (bool, default false): Sanity Check: Compute and report the deviation between the input curves and the output's tracked edges, in both directions.
      DEBUG_euler (bool, default false): Sanity Check: Compute and report the Euler characteristic of each connected component of the input curve network and of the output's tracked edges, and warn if they differ.
      DEBUG_feature_retention (bool, default false): Sanity Check: count how many of the curve network's 0-dimensional features -- open polyline endpoints and junctions -- still have a mesh vertex within eps of them, and how far the worst one that does not is.
      DEBUG_disable_envelope (bool, default false): Diagnostic only.
      DEBUG_envelope_sanity_check (bool, default false): Sanity Check: verify at init that every constrained edge of the arrangement already lies inside the envelope, and abort if one does not.
      length_rel (float, default 0.05): Target edge length relative to the bounding box diagonal.
      stop_energy (float, default 100): Target energy.
      split_high_valence_threshold (int, default 0): Incident-triangle count above which a vertex accepts only one valence-increasing split per split pass, or 0 to disable.
      project_line_search_steps (int, default 12): Bisections tried by the projected line search before it gives up on a vertex: the step along the Newton direction is halved this many times, each candidate projected onto the input, and the first that does not invert and lowers the worst incident element is taken.
      project_line_search_nested_steps (int, default 0): After the projected line search gives up, revisit each candidate and bisect between the interpolated point and its projection this many times, taking the longest step toward the input that still does not invert and still lowers the worst element.
      smoothing_mode (string, default "projected"): How smoothing places a surface vertex.
      w_amips (float, default 0.0001): Relative weight of the AMIPS quality term against the envelope term during smoothing; the envelope weight is 1 - w_amips.
      num_smoothing_passes (int, default 2): Number of smoothing passes per optimization iteration.
      interleaved_smoothing (bool, default true): Interleave smoothing between the topology passes: split + smoothing, collapse + smoothing, swaps + smoothing, instead of split, collapse, swaps followed by num_smoothing_passes smoothing passes.
      interleaved_smoothing_passes (int, default 1): Smoothing passes to run after each topology pass when interleaved_smoothing is set.
      colored_smoothing (bool, default true): Parallel smoothing (num_threads > 0) by color class: the vertices are split into classes of pairwise non-adjacent vertices, smoothed one class after another, each class fully in parallel without locks.
      order2_envelope_ratio (float, default 1.0): Radius of the anchors of the polyline endpoints, as a fraction of eps.
      feature_envelope_ratio (float, default 1.0): Radius of the anchors of the input free points (see order2_envelope_ratio), as a fraction of eps.
      preserve_topology (bool, default false): Preserve the topology of the input curve network.
      throw_on_fail (bool, default false): Throw exception if the output does not fulfil the desired criteria.
      log_file (string, default ""): Logs are not just printed on the terminal but also saved in this file.
      history_output (string, default ""): Record every change to the tracked curves, from the input files as read to the output, and save the history here (see wmtk/history): the tracking component rebuilds the map between the two from it.
      report (string, default ""): A JSON file that stores information about the result and the method execution, e.g., runtime.
      write_vtu (bool, default true): Write not just MSH but also VTU output.
      DEBUG_output (bool, default false): Write the mesh as debug_{}.vtu after every operation.
      DEBUG_sanity_checks (bool, default false): Perform sanity checks after every operation.
      max_expected_iterations (int, default 0): Fail the run if mesh_improvement needed more than this many iterations, even if it did reach stop_energy.
      stuck_refine_stall_eps (float, default 0.1): Refine when the last iteration's improvement was at most this fraction of the distance the max energy still has to cover: (prev_max - max) <= eps * (max - stop_energy).
      stuck_refine_cooldown (int, default 1): After a refinement, skip this many improvement iterations before refining again (let the operations act on the new sizing field).
      stuck_refine_num_worst (int, default 0): Number of worst triangles whose neighborhoods are refined when stuck.
      stuck_refine_rings (int, default 0): Graph rings around each worst triangle included in the stuck refinement.
      stuck_refine_factor (float, default 0.5): Multiplicative reduction of the sizing scalar per stuck refinement (0.5 => halve).
      stuck_refine_min_scalar (float, default 0.001): Lower bound on the sizing scalar during stuck refinement.
      stuck_refine_gradation (float, default 2.0): Gradation cap for the monotone sizing smoothing: adjacent sizings may differ by at most this factor (smoothing only lowers values).
      stuck_refine_force_split (bool, default true): When the max energy stalls, split each worst triangle's longest edge once, bypassing the split length gate, to unstick a sliver without changing the sizing field.
      skip_good_regions (bool, default false): Only smooth vertices incident to a triangle whose energy is at least skip_good_regions_margin * stop_energy; skip smoothing already-good regions.
      skip_good_regions_margin (float, default 0.9): A triangle is 'active' (worth smoothing around) when its energy is at least this fraction of stop_energy.
      coarsen_pass (bool, default false): Run a final coarsening pass after the optimization has converged.
      coarsen_local_smoothing_passes (int, default 0): Smoothing sweeps over the ring, inside each candidate collapse, before judging it.
      coarsen_smooth_ring (int, default 1): Radius smoothed inside the collapse, and the radius the accept test measures over.
      coarsen_global_smoothing_passes (int, default 1): Ordinary whole-mesh smoothing passes run between coarsening rounds.
      coarsen_max_rounds (int, default 2): Cap on the coarsen/smooth alternation; the pass also stops as soon as a round accepts no collapse.
      coarsen_max_inner_passes (int, default 1): Cap on the collapse pass's own dirty-epoch retry loop within one coarsening round; 0 is uncapped.
      coarsen_unbounded (bool, default true): Coarsen as far as the quality guarantee allows, instead of stopping at the target edge length.
    
    With meshes: run on named NumPy mesh dictionaries and return (meshes, report) without file I/O. Component calls run one at a time per process.
    """
@typing.overload
def triwild(options: dict) -> None:
    """
    Without meshes: run on files, as wmtk_app does with the same options.
    """
@typing.overload
def wildmeshing(options: dict) -> None:
    """
    Wildmeshing-Toolkit application
    """
@typing.overload
def wildmeshing(options: dict, meshes: dict) -> tuple:
    """
    Dispatch an in-memory component using options['application'].
    """
def write_history(path: os.PathLike | str | bytes, history: bytes) -> None:
    """
    Save the bytes of a history (a memory call's report['history'], or report['volume_history']) as the file the file route writes.
    """
def write_mesh(path: os.PathLike | str | bytes, mesh: dict) -> None:
    """
    Save MeshData through the shared MSH or existing VTU writer.
    """
