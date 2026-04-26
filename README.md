# geofikpy

[![Actions Status][actions-badge]][actions-link]
[![Documentation Status][rtd-badge]][rtd-link]

[![PyPI version][pypi-version]][pypi-link]
[![Conda-Forge][conda-badge]][conda-link]
[![PyPI platforms][pypi-platforms]][pypi-link]

[![GitHub Discussion][github-discussions-badge]][github-discussions-link]

[![Coverage][coverage-badge]][coverage-link]

<!-- SPHINX-START -->

<!-- prettier-ignore-start -->
[actions-badge]:            https://github.com/greinerth/geofikpy/actions/workflows/ci.yml/badge.svg
[actions-link]:             https://github.com/greinerth/geofikpy/actions
[conda-badge]:              https://img.shields.io/conda/vn/conda-forge/geofikpy
[conda-link]:               https://github.com/conda-forge/geofikpy-feedstock
[github-discussions-badge]: https://img.shields.io/static/v1?label=Discussions&message=Ask&color=blue&logo=github
[github-discussions-link]:  https://github.com/greinerth/geofikpy/discussions
[pypi-link]:                https://pypi.org/project/geofikpy/
[pypi-platforms]:           https://img.shields.io/pypi/pyversions/geofikpy
[pypi-version]:             https://img.shields.io/pypi/v/geofikpy
[rtd-badge]:                https://readthedocs.org/projects/geofikpy/badge/?version=latest
[rtd-link]:                 https://geofikpy.readthedocs.io/en/latest/?badge=latest
[coverage-badge]:           https://codecov.io/github/greinerth/geofikpy/branch/main/graph/badge.svg
[coverage-link]:            https://codecov.io/github/greinerth/geofikpy

<!-- prettier-ignore-end -->

Python wrapper for [GeoFIK](https://github.com/PabloLopezCustodio/GeoFIK) C++
library. The wrapped functions behave almost the same as the original
implementation; there are some exceptions. The C++ library here is compiled
against [Eigen 5.0](https://libeigen.gitlab.io/releases/5.0/). For further
details checkout the preprint of the paper
[here](https://arxiv.org/abs/2503.03992v1).

## Installation

To install the package simply execute the following command:

```bash
pip install geofikpy
```

## Usage

Here is a brief snippet on how to use the package. For more details have a look
at the unit tests.

```python
import numpy as np
from geofikpy import franka_fk, franka_ik_q4

# joint configuration
q = np.array([0.0, -np.pi / 4.0, 0.0, -3 * np.pi / 4.0, 0.0, np.pi / 2.0, np.pi / 4.0])

# get SE(3) end effector pose from joint configuration
ee_pose = franka_fk(q)

# filter solutions
_sols, idx = franka_ik_q4(ee_pose[:3, -1], np.ravel(ee_pose[:3, :3]), -3 * np.pi / 4.0)
sols = np.array([sol for sol in _sols if not np.any(np.isnan(sol))])
```

## Development Environment

Before developing, ensure that libeigen3 is installed. Using the latest
[Eigen 5.0](https://libeigen.gitlab.io/releases/5.0/), download the contents and
execute

```bash
tar -xf eigen-5.0.1.tar.bz2
cd eigen-5.0.1.tar.bz2
sudo cp -r Eigen /usr/local/include/
```

For development a virtual environment is recommendend such as
[uv](https://docs.astral.sh/uv/getting-started/installation/). Within the
virtual envronenment navigate to the project directory and execute

```bash
uv pip install -e . --group dev
```

To execute the unit tests run

```bash
nox -s tests
```
