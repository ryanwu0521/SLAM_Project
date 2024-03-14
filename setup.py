# setup.py

import os
import sys
import re
import glob
from setuptools import setup, Extension
import pybind11

# Set the name of the module and the source files
module_name = "cTheia"
source_files = glob.glob("src/isaac_sim_theia/pybind11/*.cpp")
source_files.append("src/isaac_sim_theia/jsoncpp/jsoncpp.cpp")

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