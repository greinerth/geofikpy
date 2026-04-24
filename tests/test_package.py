from __future__ import annotations

import importlib.metadata

import geofikpy as m


def test_version() -> None:
    assert importlib.metadata.version("geofikpy") == m.__version__
