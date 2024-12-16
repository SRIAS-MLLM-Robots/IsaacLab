import os
import toml

from setuptools import setup

# Obtain the extension data from the extension.toml file
EXTENSION_PATH = os.path.dirname(os.path.realpath(__file__))
# Read the extension.toml file
EXTENSION_TOML_DATA = toml.load(os.path.join(EXTENSION_PATH, "config", "extension.toml"))

# Minimum dependencies required prior to installation
INSTALL_REQUIRES = [
    # generic
    "numpy<2"
]

PYTORCH_INDEX_URL = ["https://download.pytorch.org/whl/cu118"]

# Installation operation
setup(
    name="tju-benchmark-ros2control",
    author="Tongji University Shanghai Researching Institute of Autonomous Systems & NVIDIA HER",
    maintainer="Tongji University Shanghai Researching Institute of Autonomous Systems & NVIDIA HER",
    url=EXTENSION_TOML_DATA["package"]["repository"],
    version=EXTENSION_TOML_DATA["package"]["version"],
    description=EXTENSION_TOML_DATA["package"]["description"],
    keywords=EXTENSION_TOML_DATA["package"]["keywords"],
    # license="BSD-3-Clause",
    include_package_data=True,
    python_requires=">=3.10",
    install_requires=INSTALL_REQUIRES,
    dependency_links=PYTORCH_INDEX_URL,
    packages=["omni.isaac.lab"],
    classifiers=[
        "Natural Language :: English",
        "Programming Language :: Python :: 3.10",
        "Isaac Sim :: 4.2.0",
    ],
    zip_safe=False,
)
