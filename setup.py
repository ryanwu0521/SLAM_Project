# setup.py

import os
import sys
import re
from setuptools import setup, Extension
import pybind11

# Set the name of the module and the source files
module_name = "TraversalGraph"
source_files = ["src/isaac_sim_theia/pybind11/TraversalGraph.cpp", "src/isaac_sim_theia/pybind11/TraversalNode.cpp", "src/isaac_sim_theia/pybind11/TraversalEdge.cpp","src/isaac_sim_theia/jsoncpp/jsoncpp.cpp"]

# Create the extension module
extension = Extension(
    name=module_name,
    sources=source_files,
    include_dirs=["src/isaac_sim_theia/jsoncpp", pybind11.get_include()],
    language="c++"
)

# Perform the setup
setup(
    name=module_name,
    version="1.0",
    author="Your Name",
    author_email="your.email@example.com",
    description="Python bindings for TraversalGraph using pybind11",
    ext_modules=[extension]
)
