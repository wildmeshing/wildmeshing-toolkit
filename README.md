# Wildmeshing-toolkit: Declarative Specification for Unstructured Mesh Editing Algorithms

## Installation

#### via CMake

Our code was originally developed on MacOS and has been tested on Linux and Windows. We provide the commands for installing in MacOS:

- Clone the repository into your local machine:

```bash
git clone https://github.com/wildmeshing/wildmeshing-toolkit.git
```

- Compile the code using cmake (default in Release mode):

```bash
cd wildmeshing-toolkit
mkdir build
cd build
cmake ..
make
```

You may need to install `gmp` and `continuous.yml` before compiling the code. You can install `gmp` via [homebrew](https://brew.sh/).

```
brew install gmp
```
## Usage
This toolkit is a novel approach to describe mesh generation, mesh adaptation, and geometric modeling algorithms relying on changing mesh connectivity using a high-level abstraction. The main motivation is to enable easy customization and development of these algorithms via a declarative specification consisting of a set of per-element invariants, operation scheduling,and attribute transfer for each editing operation.

Many widely used algorithms editing surfaces and volumes
can be compactly expressed with our abstraction, and their implementation
within our framework is simple, automatically parallelizable on shared-
memory architectures, and with guaranteed satisfaction of the prescribed
invariants. These algorithms are readable and easy to customize for specific use cases.

This software library implements the abstractiona in our paper and providing automatic shared memory parallelization.

We will introduce the framework and run through the basic software structures and APIs with a step-by-step guide to a simple implementation of the shortest edge collapse algorithm. 

## Basic Algorithm

## Explicit Invariant Design

## Operation Rollback

## Explicit Attribute Update 

## Parallel Scheduling
The type and scheduling of local operations is crucial in mesh editing algorithms. This involves maintaining a priority queue of operations, which is updated after every local operation.
operation.
- Queue maintenance
- Operation selection
- Operation initiation (see Abstract Mesh Navigation)

## Abstract Mesh Navigation




### Command Line Switches
Our software supports usage via command line or via a C++ function wrapper. Here is an overview of all command line switches:

```
RobustTetMeshing
Usage: ./TetWild [OPTIONS] input [output]

Positionals:
  input TEXT REQUIRED         Input surface mesh INPUT in .off/.obj/.stl/.ply format. (string, required)
  output TEXT                 Output tetmesh OUTPUT in .msh format. (string, optional, default: input_file+postfix+'.msh')

Options:
  -h,--help                   Print this help message and exit
  --input TEXT REQUIRED       Input surface mesh INPUT in .off/.obj/.stl/.ply format. (string, required)
  --output TEXT               Output tetmesh OUTPUT in .msh or .mesh format. (string, optional, default: input_file+postfix+'.msh')
  --postfix TEXT              Postfix P for output files. (string, optional, default: '_')
  -l,--ideal-edge-length FLOAT
                              ideal_edge_length = diag_of_bbox * L. (double, optional, default: 0.05)
  -e,--epsilon FLOAT          epsilon = diag_of_bbox * EPS. (double, optional, default: 1e-3)
  --stage INT                 Run pipeline in stage STAGE. (integer, optional, default: 1)
  --filter-energy FLOAT       Stop mesh improvement when the maximum energy is smaller than ENERGY. (double, optional, default: 10)
  --max-pass INT              Do PASS mesh improvement passes in maximum. (integer, optional, default: 80)
  --is-laplacian              Do Laplacian smoothing for the surface of output on the holes of input (optional)
  --targeted-num-v INT        Output tetmesh that contains TV vertices. (integer, optional, tolerance: 5%)
  --bg-mesh TEXT              Background tetmesh BGMESH in .msh format for applying sizing field. (string, optional)
  --save-mid-result           0: save result before optimization, 1: save mid-results during optimization, 2: save result without winding number.
  -q,--is-quiet               Mute console output. (optional)
  --log TEXT                  Log info to given file.
  --level INT                 Log level (0 = most verbose, 6 = off).
```

<!--### Tips
TODO :)-->

### Function Wrapper

💡 We use [libigl](https://github.com/libigl/libigl) to read the input triangle mesh. If you encounter any issue loading your mesh with libigl, please open a ticket there. Alternatively, you could load the mesh yourself and use our function wrapper to pass the raw data directly to TetWild.

We provide a wrapper for TetWild in `tetwild.h`, allowing users do the tetrahedaliztion without read/write data from/to files. One can use it in the following way:

1. Include the header file `#include <tetwild/tetwild.h>`.
2. Set parameters through a struct variable `tetwild::Args args`. The following table provides the correspondence between parameters and command line switches.

	| Switch              | Parameter                   |
	|:--------------------|:----------------------------|
	| --input             | N/A                         |
	| --postfix           | `args.postfix`              |
	| --output            | N/A                         |
	| --ideal-edge-length | `args.initial_edge_len_rel` |
	| --epsilon           | `args.eps_rel`              |
	| --stage             | `args.stage`                |
	| --filter-energy     | `args.filter_energy_thres`  |
	| --max-pass          | `args.max_num_passes`       |
	| --is-quiet          | `args.is_quiet`             |
	| --targeted-num-v    | `args.target_num_vertices`  |
	| --bg-mesh           | `args.background_mesh`      |
	| --is-laplacian      | `args.smooth_open_boundary` |

3. Call function `tetwild::tetrahedralization(v_in, f_in, v_out, t_out, a_out, args)`. The input/output arguments are described in the function docstring, and use libigl-style matrices for representing a mesh.

## License
TetWild is MPL2 licensed. But it contains CGAL code under GPL license. 

TetWild is free for both commercial and non-commercial usage. However, you have to cite our work in your paper or put a reference of TetWild in your software. Whenever you fix bugs or make some improvement of TetWild, you should contribute back.

## Acknowledgements

We used several useful libraries in our implement, testing, and rendering listed as follows. We would like to especially thank their authors for their great work and publishing the code.

- [PyMesh](https://github.com/qnzhou/PyMesh)
- [PyRenderer](https://github.com/qnzhou/PyRenderer)
- [CLI11](https://github.com/CLIUtils/CLI11)
