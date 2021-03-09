#!/usr/bin/env python

import pytest
import pyadrc

@pytest.fixture
def quad():
    def _quad(dt=0.001, m=0.028, g=9.807):
        return pyadrc.QuadAltitude(dt, m, g)
    return _quad


def test_direction(quad):
    counter = 0
    quad = quad(dt=0.01)
    while counter < 5:
        quad(10)
        counter += 0.01

    pos, _, _ = quad.states

    assert pos > 0


def test_ground(quad):
    quad = quad(dt=0.01)
    quad.states = (5, 0, 0)

    counter = 0
    while counter < 5:
        quad(0)
        counter += 0.01

    pos, vel, acc = quad.states

    assert pos >= 0 and vel == 0 and acc < 0
