#!/usr/bin/env python

"""Tests for `pyadrc.TransferFunction` module."""

import pytest
import pyadrc
import time
import numpy as np
from scipy import signal


@pytest.fixture
def create_tf():
    def _create_tf(order, delta, b0, w_cl, k_eso, method):
        tf = pyadrc.TransferFunction(order, delta, b0, w_cl,
                                     k_eso, method)
        return tf
    return _create_tf


@pytest.mark.parametrize("order, delta, b0, w_cl, k_eso", [
    (1, 0.001, 1, 1, 1),
    (1, 0.001, 0.1, 1, 1),
    (1, 0.001, 0.5, 1, 1),
    (1, 0.001, 1.1, 1, 1),
    (1, 0.001, 1.5, 1, 1),
    (1, 0.001, 1, 0.1, 1),
    (1, 0.001, 1, 0.5, 1),
    (1, 0.001, 1, 1.1, 1),
    (1, 0.001, 1, 1.5, 1),
    (1, 0.001, 1, 1, 3),
    (1, 0.001, 1, 1, 5),
    (1, 0.001, 1, 1, 8),
    (1, 0.001, 1, 1, 10),
    (2, 0.001, 1, 1, 1),
    (2, 0.001, 0.1, 1, 1),
    (2, 0.001, 0.5, 1, 1),
    (2, 0.001, 1.1, 1, 1),
    (2, 0.001, 1.5, 1, 1),
    (2, 0.001, 1, 0.1, 1),
    (2, 0.001, 1, 0.5, 1),
    (2, 0.001, 1, 1.1, 1),
    (2, 0.001, 1, 1.5, 1),
    (2, 0.001, 1, 1, 3),
    (2, 0.001, 1, 1, 5),
    (2, 0.001, 1, 1, 8),
    (2, 0.001, 1, 1, 10)
])
def test_check_param_method(create_tf,
                            order, delta, b0,
                            w_cl, k_eso):

    tf1 = create_tf(order, delta, b0, w_cl, k_eso, 'general_terms')
    tf2 = create_tf(order, delta, b0, w_cl, k_eso, 'bandwidth')
    assert tf1.params == tf2.params
