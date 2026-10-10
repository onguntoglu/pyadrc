#!/usr/bin/env python

"""Tests for `pyadrc.TrackingDifferentiator` and `pyadrc.fhan`."""

import numpy as np
import pytest

import pyadrc


def _step(r0, delta, h0=None, step=1., seconds=5.):
    td = pyadrc.TrackingDifferentiator(delta, r0, h0)
    v = np.array([td(step) for _ in range(int(seconds / delta))])
    return v[:, 0], v[:, 1]


@pytest.mark.parametrize("r0, delta", [
    (10., 0.01),
    (50., 0.001),
    (2., 0.1),
    (5., 0.001),
])
@pytest.mark.parametrize("step", [1., -0.5])
def test_time_optimal_step(r0, delta, step):
    """Han (2009): fastest convergence without overshoot, |dv2/dt| <= r0.
    Fails for the printed d = h0 * r0**2 in fhan."""

    v1, v2 = _step(r0, delta, step=step)

    # Acceleration limit
    assert np.abs(np.diff(v2)).max() <= r0 * delta * (1 + 1e-9)

    # No overshoot (up to discretization at the switching point)
    assert (v1 * np.sign(step)).max() - abs(step) <= r0 * delta**2

    # Settles near the time-optimal bang-bang time 2 * sqrt(|step| / r0)
    t_opt = 2 * np.sqrt(abs(step) / r0)
    settled = np.abs(v1 - step) < 1e-3 * abs(step)
    t_settle = delta * np.argmax(settled)
    assert settled[-1] and t_settle <= t_opt + 3 * delta
    assert v1[-1] == pytest.approx(step, abs=1e-6)
    assert v2[-1] == pytest.approx(0, abs=1e-6)


def test_fhan_limits():

    r0, h0 = 10., 0.01
    x1, x2 = -1., 0.

    # Far from the target, fhan is bang-bang at the acceleration limit
    assert pyadrc.fhan(x1, x2, r0, h0) == pytest.approx(r0)
    # At the target, it stays there
    assert pyadrc.fhan(0., 0., r0, h0) == 0.


def test_constant_reference():

    td = pyadrc.TrackingDifferentiator(0.01, 10.)
    td.reset(v1=2.)
    for _ in range(100):
        v1, v2 = td(2.)
    assert (v1, v2) == (2., 0.)
    assert td.states == (2., 0.)


def test_filter_factor_slows_down():
    """A larger h0 gives a slower transient, still without overshoot"""

    def settle(v1):
        return np.argmax(np.abs(v1 - 1.) < 1e-3)

    v1_fast, _ = _step(10., 0.01)
    v1_slow, _ = _step(10., 0.01, h0=0.1)

    assert settle(v1_slow) > settle(v1_fast)
    assert v1_slow.max() <= 1.
    assert v1_slow[-1] == pytest.approx(1.)


def test_derivative_of_sine():

    delta, w = 0.001, 2 * np.pi * 0.5
    td = pyadrc.TrackingDifferentiator(delta, r0=1000.)
    t = np.arange(0, 4, delta)
    v2 = np.array([td(np.sin(w * tk))[1] for tk in t])

    # Ignore the initial transient
    np.testing.assert_allclose(v2[1000:], (w * np.cos(w * t))[1000:],
                               atol=0.02)


def test_invalid_arguments():

    with pytest.raises(ValueError):
        pyadrc.TrackingDifferentiator(0, 1.)
    with pytest.raises(ValueError):
        pyadrc.TrackingDifferentiator(0.01, -1.)
    with pytest.raises(ValueError):
        pyadrc.TrackingDifferentiator(0.01, 1., h0=0)
