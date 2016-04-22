# Robust Edge types for g2o

This package provides edge types in g2o for use in robust optimization. Specifically, it contains:

- **MaxMixture** edges, implemented by Praktik Agarwal, from https://github.com/agpratik/max-mixture, licensed under the BSD
- **SwitchableConstraints** edges, from the Vertigo package by Niko Suenderhauf, from https://svn.openslam.org/data/svn/vertigo/, licensed under the GPLv3
- **RobustifiedEdges**: Generic robust edges allowing the per-edge use of robust kernels from g2o, released in public domain via CC0

Please see the individual README and LICENSE files the the subdirectories for more information.

## Installation

### Dependencies

- G2O, git version from https://github.com/RainerKuemmerle/g2o
- Eigen3 (also a G2O dependency)

### Compiling

Compile with

```
mkdir ./build
cd ./build
cmake ..
make
```

Use `ccmake` or `cmake-gui` to have more control over build variables, like the `CMAKE_BUILD_TYPE` or where G2O is found.

If G2O can't be found, try setting an environment variable to where you installed G2O called `G2O_ROOT`.

### Installation

```
sudo make install
```
will install to the location you specified with `CMAKE_INSTALL_PREFIX`.


## Running

Once the plugins are built or installed, you can run for example the `g2o` command line program with these custom edge types using the `-typeslib` option.

To run the robustified edges example:

```
LD_LIRBARY_PATH=${G2O_ROOT}/lib ${G2O_ROOT}/bin/g2o -v -typeslib RobustifiedEdges/libg2o_types_robust_edges.so ../examples/robustified_se2.g2o
````

## Troubleshooting

### Robust Edges don't seem to be robustified.

Check for lines starting with "EdgeRobustified::read: unknown kernel", which happens if you misspell the robust kernel to use. Unfortunately, G2O does not check the return value of `g2o::HyperGraph::Edge::read()`, so they appear to load fine, but the kernel is not set.

