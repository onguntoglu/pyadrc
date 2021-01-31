#!/usr/bin/env python

"""Tests for `pyadrc` package."""

import pytest
import pyadrc


@pytest.fixture
def check_saturation() -> float:
    def _sat(_limits, _val: float):
        return pyadrc.saturation(_limits, _val)
    return _sat


def test_check_saturation(check_saturation):

    # Standard check
    assert check_saturation((0, 100), 120) == 100
    assert check_saturation((0, 100), -20) == 0
    assert check_saturation((0, 100), 42) == 42

    # No limits given, i.e. (None, None)
    assert check_saturation((None, None), 120) == 120
    assert check_saturation((None, None), -20) == -20
    assert check_saturation((None, None), 42) == 42

    # Lower limit, no upper limit
    assert check_saturation((-5, None), -10) == -5
    assert check_saturation((-5, None), -5) == -5
    assert check_saturation((-5, None), 0) == 0

    # Upper limit, no lower limit
    assert check_saturation((None, 5), 10) == 5
    assert check_saturation((None, 5), 5) == 5
    assert check_saturation((None, 5), -10) == -10
