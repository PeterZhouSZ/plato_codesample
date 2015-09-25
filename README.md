# Customizable Objects for Fabrication

This repository contains a heavily redacted version of the code for the  SIGGRAPH 2015 paper "Fab Forms: Customizable Objects for Fabrication with Validity and Geometry Caching". All of the code was written by Masha Shugrina, with the exception of a few javascript snippets that were ported from the V0 UI implementation by Sashko Stubailo. This repository is made public merely for illustrative purposes. All code is copyrighted.

### Overview
The goal of this project was to convert general parametric designs, represented here as __geometry templates__, into a representation that can be easily customized by a novice while maintaining high-level design validity. To this end, each geometry template is authored by a skilled user in a __desktop authoring tool__, where it is annotated with validity tests. Then, valid regions of the design space are approximated during __precomputation__ on Amazon Web Services. Finally, precomputed data structure are used to automatically spawn a __web customization UI__ that guards the user against creating invalid designs and provides interactive preview even for expensive geometry. The code can be broken down into the following broad categories:
  - __Geometry Templates__ [`fab/geometry/templates`]- flexible representation of complex parametric shapes as a tree of arbitrary operation notes with arbitrary parameters linked by constraints in C++; validity tests.
  - __Precomputation__ [`fab/geometry/templates/precomp`] - C++ data structures, logic and bash scripts for AWS that sample the design space and store the results of tests and geometry generation.
  - __Generator of Web Customizer UI__ [`fab/plato_web2`] - C++, javascript code that automatically creates a Web customization UI, given a template and precomputation results
  - __Desktop Authoring Tool__ [`fab/qt_templatizer`] - GUI for designing geometry templates (most code removed)
  - __Utilities__ [`fab/geometry`, `util`] - geometry processing and other utilities

### About Removed Code
Most headers have been kept intact, and most implementation files removed. The code that is left still illustrates the architecture of the system, and the few implementation files show the general coding style and standards. CMakeLists.txt files have mostly been left unchanged.

### Walkthrough of the Software Design
Constrained parametric designs can be represented as a tree of operation nodes, where some shape information, such as a triangle mesh, travels along the edges from the leaves to the root and is modified and merged by internal nodes. Each node is governed by any number of typed named parameters (e.g. int, double) that can be linked by any number arithmetic constraints (we do not cover geometric constraints).

The original goal was to architect this system in C++ such that it is completely agnostic to the operation nodes used, the types of parameters, the number of constraints, the number of parameters exposed to the end user (i.e. dimensionality of the design space). In retrospect, this was probably the wrong decision for a research project, but this would be the way to go for a commercial library.

__Geometry template__ `FabGeometryTemplate` is defined in `template.h`. Its tree of operation nodes, constraints and checks can be written to file and read back (e.g. `data/fab/plato_examples`). This spec is defined as a Google protocol buffer in `template.proto` (this automatically generates corresponding C++ classes and handles I/O). Each `OperationNode` can contain any `Operation` subclass (`operation.h`), and new operations subclasses (such as those in the `operations/` subdirectory) can be used with the code by simply statically linking them with the library (via a registration mechanism in `registration.h`).

__Parameters__ of different types (double, int, bool) in the geometry template can be retrieved and set by name. This also allows to write constraints as expressions and store them in the template. Constraints are implemented using the free Gecode library, and expression parsing is implemented in `constraints/` subdirectory.

During precomputation, the result of design space precomputation is stored in a `PrecompLookup` data structure (see `precomp/lookup.h`), which is can be subclassed as a `GridLookup` for the initial sampling over a uniform grid, or `HybridLookup`, which is a grid sampled of the design space with a k-d tree of samples at every hyperrectangle. Sampling algorithm is only feasible for low-dimensional design spaces, but the code itself works for any dimensionality and intelligently handles int, bool, double dimensions (see `Dimension` in `lookup.h`). The key main function where precomputation is performed is `precomp/hadoop_mapper_main.cc` (misnomer, we were unable to use hadoop because it is not available for custom Maching Images on EC2), and all stages of precomputation are run via `precomp/scripts.sh`.


### Cut Corners

The code is unoptimized, it lacks unit tests, and there are many cut corners. I would take greater care to think through representation of data, test individual components and organize the code better. Most certainly, in a production system each class would be in a separate file (not like `lookup.h`!) and the parallel computation would not be initiated via a 1200 line shell script. It is also quite unfortunate that we could not rely on automatic parallel computation frameworks such as hadoop, as it was not available for custom Machine Images on Amazon EC2, which we needed.
