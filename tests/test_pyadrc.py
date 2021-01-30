#!/usr/bin/env python

"""Tests for `pyadrc` package."""

import pytest
import pyadrc


@pytest.fixture
def quadcopter_model():
    return pyadrc.QuadAltitude()

@pytest.fixture
def saturation(_val: float) -> float:
    return pyadrc.saturation((0, 100), _val)


def check_saturation(saturation) -> float:
    assert saturation(120) == 100
    assert saturation(-20) == 0
    mid = 42.
    assert saturation(mid) == mid


def test_content(response):
    """Sample pytest test function with the pytest fixture as an argument."""
    # from bs4 import BeautifulSoup
    # assert 'GitHub' in BeautifulSoup(response.content).title.string
