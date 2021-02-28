#!/usr/bin/env python

"""Tests for `pyadrc` package."""

import pytest
import pyadrc
import time
import numpy as np


@pytest.fixture
def check_saturation() -> float:
    def _sat(_limits, _val: float):
        return pyadrc.saturation(_limits, _val)
    return _sat


@pytest.fixture
def adrc_ss():
    def _adrc_ss(order, delta, b0, t_settle, k_eso, eso_init: np.array = False,
                 r_lim: tuple = (None, None),
                 m_lim: tuple = (None, None),
                 half_gain: tuple = (False, False)):
        return pyadrc.StateSpace(
            order, delta, b0, t_settle, k_eso, eso_init, r_lim, m_lim, half_gain)
    return _adrc_ss


@pytest.fixture
def adrc_ss_nominal(adrc_ss):
    return adrc_ss(1, 1, 1, 1, 1)


@pytest.mark.parametrize("limits, val, expected", [
    ((0, 100), 120, 100),
    ((0, 100), -20, 0),
    ((0, 100), 42, 42),
    ((None, None), 120, 120),
    ((None, None), -20, -20),
    ((None, None), 42, 42),
    ((-5, None), -10, -5),
    ((-5, None), -5, -5),
    ((-5, None), 0, 0),
    ((None, 5), 10, 5),
    ((None, 5), 5, 5),
    ((None, 5), -10, -10)
])
def test_check_saturation(check_saturation, limits, val, expected):
    assert check_saturation(limits, val) == expected


def test_zero(adrc_ss_nominal):

    assert adrc_ss_nominal(1, 1, 0, True) == 0.


@pytest.mark.parametrize("y, r, b0", [
    (0.5, 1, 1),
    (-0.5, 1, 1),
    (0.5, -1, 1),
    (-0.5, -1, 1),
    (0.5, 1, -1),
    (-0.5, 1, -1),
    (0.5, -1, -1),
    (-0.5, -1, -1)
])
def test_direction(adrc_ss, y, r, b0):

    """direction of control action w.r.t. modeling parameter b0, reference\
    r and current output y"""
    adrc = adrc_ss(order=1, delta=1, b0=b0, t_settle=1, k_eso=1)
    assert np.sign(adrc(y, 0, r)) == np.sign(r - y) * np.sign(b0)


def test_zoh(adrc_ss_nominal):
    # Test zero-order hold
    u = 0
    y = 0

    # Get control action calculated by ADRC
    u = adrc_ss_nominal(y=y, u=u, r=10, zoh=True)

    # Wait 0.5 seconds, controller should return the same value
    time.sleep(0.5)
    ret1 = adrc_ss_nominal(y=y, u=u, r=10, zoh=True)

    assert u == ret1

    # Wait another 0.5 seconds, controller should return new value
    time.sleep(0.5)
    ret2 = adrc_ss_nominal(y=y, u=ret1, r=10, zoh=True)

    assert ret1 != ret2


def test_magnitude_limit(adrc_ss):

    adrc = adrc_ss(order=1, delta=1, b0=10, t_settle=1, k_eso=1, m_lim=(0, 5))
    u1 = adrc(0, 0, 30)

    assert u1 == 5

    adrc.magnitude_limits = (0, 10)
    u2 = adrc(0, u1, 30)

    assert u2 == 10


def test_rate_limit(adrc_ss):

    adrc = adrc_ss(order=1, delta=1, b0=10, t_settle=1, k_eso=1, r_lim=(0, 1))

    u1 = adrc(0, 0, 30)
    u2 = adrc(0, u1, 30)

    assert (u2 - u1) == 1

    adrc.rate_limits = (0, 5)

    u3 = adrc(0, u2, 30)

    assert (u3 - u2) == 5
