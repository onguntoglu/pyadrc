#!/usr/bin/env python

"""Tests for `pyadrc` package."""

import pytest
import pyadrc

import numpy as np


@pytest.fixture
def check_saturation() -> float:
    def _sat(_limits, _val: float):
        return pyadrc.saturation(_limits, _val)
    return _sat


@pytest.fixture
def check_common_loc_obs_poles():
    """Common location observer poles
    """
    def _adrc(order, delta, b0, t_settle, k_eso):
        ctrl = pyadrc.adrc.state_space(order, delta, b0, t_settle, k_eso)
        poles = np.linalg.eigvals(ctrl.Ad - ctrl.L @ ctrl.Cd)
        return poles
    return _adrc


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


def test_common_location_observer_poles(check_common_loc_obs_poles):
    # order = 1, delta = 0.001, b0 = 1, t_settle = 0.25, k_eso=1, 5, 10
    check_common_loc_obs_poles(1, 0.001, 1, 0.25, 1)
    check_common_loc_obs_poles(1, 0.001, 1, 0.25, 5)
    check_common_loc_obs_poles(1, 0.001, 1, 0.25, 10)
