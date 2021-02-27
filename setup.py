# -*- coding: utf-8 -*-
#
# This file is part of the motorlib project
#
# Copyright (c) 2021 Tiago Coutinho
# Distributed under the GPLv3 license. See LICENSE for more info.

"""The setup script."""

import sys
from setuptools import setup, find_packages

with open("README.md") as f:
    readme = f.read()

test_requirements = ["pytest", "pytest-cov"]

setup_requirements = []

needs_pytest = {"pytest", "test"}.intersection(sys.argv)
if needs_pytest:
    setup_requirements.append("pytest-runner")

setup(
    name="motorlib",
    author="Jose Tiago Macara Coutinho",
    author_email="coutinhotiago@gmail.com",
    classifiers=[
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Intended Audience :: System Administrators",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Natural Language :: English",
        "Operating System :: POSIX",
        "Programming Language :: Python :: 3.5",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
    ],
    description="motor motion/trajectory simulation library",
    license="GPLv3",
    long_description=readme,
    long_description_content_type="text/markdown",
    include_package_data=True,
    keywords="motor,axis,motion,trajectory,simulation",
    py_modules=["motorlib"],
    setup_requires=setup_requirements,
    test_suite="tests",
    tests_require=test_requirements,
    python_requires=">=3.5",
    url="https://gitlab.com/tiagocoutinho/motorlib",
    version="0.1.0",
    zip_safe=True,
)
