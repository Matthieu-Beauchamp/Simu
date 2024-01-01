# Cpp Read The Docs

This repo serves as a base configuration for using the doxygen-breathe-sphinx 
pipeline for documenting C++ code and producing HTML documentation from 
Doxygen comments and Sphinx's reStructuredText markup pages.

Features:
- Plug and play pipeline from Doxygen comments to Read the Docs themed documentation
- Doxygen style documentation in the code (support depends on [Breathe](https://www.breathe-doc.org/))
- reStructuredText documentation in the code (see [Documenting the code](#documenting-the-code))
- Sphinx's reStructuredText markup documentation pages
- Build documentation with a CMake target


# Installation

You will need to have installed on your machine
- [Doxygen](https://www.doxygen.nl/download.html)
- [Python 3](https://www.python.org/downloads/)


Install all the contents of this repo into a `docs` subdirectory of your project:
```sh
curl -J -L https://github.com/Matthieu-Beauchamp/CppReadTheDocs/archive/main.tar.gz | tar -z -x
mv CppReadTheDocs-main docs
```

# Quick start

Assuming your project is currently documented using Doxygen, most features will 
work as expected. See [Documenting the code](#documenting-the-code) for more details.

In your project's root CMakeLists.txt (or the one besides `docs`), 
paste the following lines

```cmake
set(DOCS_TARGET_NAME "Unnamed-project") # Must not have spaces
set(DOCS_PROJECT_NAME "Unnamed project") # A pretty name to show in documentation

set(DOCS_AUTHOR "Anynomous hero") # Your name
set(DOCS_PROJECT_RELEASE "0.0.1") # Your project's version

# The following are relative to the docs directory
set(DOCS_HEADER_INPUT_DIR "../include")
set(DOCS_INPUT_DIR "source")
set(DOCS_OUTPUT_DIR "build")

set(DOCS_LANGUAGE "en") # https://www.sphinx-doc.org/en/master/usage/configuration.html#confval-language
set(DOCS_FORMAT "html") # https://www.sphinx-doc.org/en/master/usage/builders/index.html

# When this is true, a ${DOCS_TARGET_NAME}-docs CMake target is available
set(BUILD_DOCS TRUE)

# The root directory of your project (absolute directory) to be removed
# in the documentation
set(DOCS_STRIP_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}")

# A comma separated list of include directories (absolute) for the proper #include 
# directives to appear in the documentation
set(DOCS_INCLUDE_DIRS "${CMAKE_CURRENT_SOURCE_DIR}/include")

add_subdirectory("docs")
```

You should now be able to build the `${DOCS_TARGET_NAME}-docs` target and see your
documentation at `docs/build/index.html`.

# Writing the documentation

## Documenting the code

Most of Doxygen features will be supported. This support entirely depends on [Breathe](https://www.breathe-doc.org/).
In case a feature does not work, the `\rst` `\endrst` block can be used to write
reStructuredText that will be passed to Sphinx.


## Source
The `source` subdirectory contains the reStructuredText (.rst) files for your
documentation, you may add as many as you want. Refer to [Sphinx reStructuredText documentation](https://www.sphinx-doc.org/en/master/usage/restructuredtext/index.html)
as well as the [C++ domain](https://www.sphinx-doc.org/en/master/usage/domains/cpp.html) and [Breathe's directives documentation](https://breathe.readthedocs.io/en/latest/directives.html) 
for writing your documentation.

Note that a `reference.rst` is already included in your sources and will contain
the entire documentation generated from headers in a single page. This can be replaced by a handmade index.


# Advanced configuration

## Sphinx

The Sphinx configuration file `source/conf.py.in` can be modified to your liking (see 
[Sphinx configuration documentation](https://www.sphinx-doc.org/en/master/usage/configuration.html)). 

__Be careful when changing fields like `@DOCS_PROJECT_NAME@` as these are set by CMake.__

__Be careful to only modify `conf.py.in` and NOT `conf.py` which will be overwritten every time CMake configures__


## Doxygen 

The `Doxyfile.in` can be modified to your liking.

__Be careful when changing fields like `@DOCS_PROJECT_NAME@` as these are set by CMake.__

__Be careful to only modify `Doxyfile.in` and NOT `Doxyfile` which will be overwritten every time CMake configures__