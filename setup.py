import os
import re
import subprocess
import sys
from pathlib import Path

from setuptools import setup, find_packages, Extension
from setuptools.command.build_ext import build_ext


setup(
    name="batbot_bringup",
    version="7.0.0",
    author="Jayson De La Vega, Ben Westcott, Mason Lopez",
    author_email="jaysond21@vt.edu",
    description="Batbot7 gui bringup application",
    package_dir={"": "src"},
    packages=find_packages(where="src"),
    python_requires=">=3.8",
    install_requires=[
        'pyserial'
    ]
)