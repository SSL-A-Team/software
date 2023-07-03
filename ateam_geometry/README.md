# A-Team Geometry Package

This package defines shared geometry utilities to be used across our code base. These utilities are implemented using [The Computational Geometry Algorithms Library](https://www.cgal.org/).

When using the library, you can either include the specific headers for the utilities you intend to use, or simply include the top level `ateam_geometry.hpp` header, which will give you access to all of our utilities.

Our geometry object types are aliases on CGAL types in the `Simple_cartesian` kernal. This means you can use all the normal CGAL algorithms and utilities to work with these objects.
